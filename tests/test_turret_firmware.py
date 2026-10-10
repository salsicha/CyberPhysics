"""Turret firmware core: host-compiled C++ unit tests and protocol cross-checks.

Builds applications/arduino/scripts/turret_firmware/test/host_test.cpp with
the shared core (src/*.cpp, the code both the UNO R4 WiFi and UNO Q sketches
run) using the host C++ compiler, runs its tests (packets against Feetech
reference bytes, limits and conversions, watchdog, STOP, servo errors against
simulated servos), and checks that the firmware and the Python bridge parse
each other's lines. Skipped without a C++ compiler.
"""
import shutil
import subprocess
import sys
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, install_ros_stubs, load_script

FIRMWARE = REPO / 'applications/arduino/scripts/turret_firmware'


@pytest.fixture(scope='module')
def host_test(tmp_path_factory):
    compiler = shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
    if compiler is None:
        pytest.skip('no C++ compiler')
    out = tmp_path_factory.mktemp('turret_firmware') / 'host_test'
    sources = [str(FIRMWARE / 'test/host_test.cpp')]
    sources += sorted(str(path) for path in (FIRMWARE / 'src').glob('*.cpp'))
    base = [compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', '-O1',
            f'-I{FIRMWARE / "src"}', *sources, '-o', str(out)]
    # Sanitizers when the toolchain has them
    result = subprocess.run(base + ['-fsanitize=address,undefined'],
                            capture_output=True, text=True, check=False)
    if result.returncode != 0:
        result = subprocess.run(base, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    return out


def run(host_test, *args):
    result = subprocess.run([str(host_test), *args], capture_output=True, text=True,
                            timeout=60, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


@pytest.fixture(scope='module')
def bridge():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors'):
            sys.modules[name] = MagicMock()
        sys.modules['serial'] = None
        yield load_script(REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py',
                          'turret_arduino_bridge_firmware')


def test_firmware_core_unit_tests(host_test):
    output = run(host_test)
    assert ' 0 failures' in output


@pytest.mark.parametrize('pan,tilt', [(0.5, -0.25), (-1.5708, 0.75), (0.0, 0.0),
                                      (1e-7, -3.0), (3.25, -0.65)])
def test_bridge_set_lines_parse_in_firmware(host_test, bridge, pan, tilt):
    line = bridge.format_command(pan, tilt)
    assert line.endswith('\n')
    kind, fw_pan, fw_tilt = run(host_test, 'parse', line.rstrip('\n')).split()
    assert kind == 'SET'
    assert float(fw_pan) == pytest.approx(pan, abs=1e-6)
    assert float(fw_tilt) == pytest.approx(tilt, abs=1e-6)


def test_bridge_stop_line_parses_in_firmware(host_test, bridge):
    assert run(host_test, 'parse', bridge.STOP_LINE.rstrip('\n')).strip() == 'STOP'
    assert run(host_test, 'parse', 'SET pan_joint nan tilt_joint 0').startswith('INVALID')


@pytest.mark.parametrize('pan,tilt', [(0.1, -0.2), (-1.5708, 0.75), (0.0, -0.0),
                                      (1.234567, -0.000001)])
def test_firmware_pos_lines_parse_in_bridge(host_test, bridge, pan, tilt):
    line = run(host_test, 'format', repr(pan), repr(tilt))
    assert line.endswith('\n')
    assert bridge.parse_position_line(line) == pytest.approx((pan, tilt), abs=1e-6)

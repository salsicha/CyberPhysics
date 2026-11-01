"""ArduPilot SITL's clock behind the Gazebo JSON backend
(applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh;
ardupilot_json.py, SitlClock; gazebo_ardupilot_backend.py).

ArduPilot learns its IMU's sample rate only in the first 30 s of SITL's
clock. In the hardware-in-the-loop setup SITL waited on its first serial
port for the Pi's MAVROS, about 25 s, and its clock came up at that
wall time: the estimate stayed near the nominal 1000 Hz for the 100 Hz the
backend delivers, the gyro filter ran ten times too slow and the attitude
loop oscillated from lift-off (2026-11-01, HIL runs gi1-gi3: 683-841 Hz at
arming, against 109-176 Hz in every all-local run; tools/aerodrone_hil_test.md).
SITL's clock starts at the later of the first state's timestamp and the
wall time SITL spent before it: the launcher keeps the wait short, SitlClock
keeps the timestamp small."""
import cmath
import math
import os
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, load_script

sys.path.insert(0, str(REPO / 'applications/ardupilot_sitl/scripts'))
from ardupilot_json import SitlClock  # noqa: E402


START = SitlClock.START_S


def test_sitl_clock_starts_at_the_first_exchange():
    clock = SitlClock()
    assert clock.timestamp(95.0, 0) == pytest.approx(START)
    assert clock.timestamp(95.0, 1) == pytest.approx(START)   # same pose, next frame
    assert clock.timestamp(95.01, 2) == pytest.approx(START + 0.01)
    assert clock.timestamp(155.0, 6000) == pytest.approx(START + 60.0)


def test_sitl_clock_never_dates_a_state_zero():
    # SITL's HAL reads a stopped clock of 0 as none and runs on wall time.
    assert 0.0 < START < 1.0
    assert SitlClock().timestamp(0.0, 0) > 0.0


def test_sitl_clock_starts_again_for_a_new_sitl_session():
    clock = SitlClock()
    clock.timestamp(20.0, 0)
    assert clock.timestamp(80.0, 24000) == pytest.approx(START + 60.0)
    # SITL restarted: its servo frame count starts again from zero.
    assert clock.timestamp(300.0, 0) == pytest.approx(START)
    assert clock.timestamp(300.5, 200) == pytest.approx(START + 0.5)


def backend_node(monkeypatch, tmp_path):
    class FakeNode:
        def __init__(self, name):
            self.create_publisher = MagicMock()
            self.create_subscription = MagicMock()
            self.create_timer = MagicMock()
            self.get_logger = MagicMock()

    modules = {name: MagicMock() for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'actuator_msgs', 'actuator_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg')}
    modules['rclpy.node'].Node = FakeNode
    with patch.dict(sys.modules, modules):
        backend = load_script(REPO / 'applications/gazebo/scripts/gazebo_ardupilot_backend.py',
                              'gazebo_backend_clock_test')
    manifest = tmp_path / 'manifest.json'
    manifest.write_text('{"home_enu_m": [0, 0, 0]}')
    monkeypatch.setenv('TERRAIN_MANIFEST', str(manifest))
    transport = MagicMock()
    monkeypatch.setattr(backend, 'ArduPilotJSON', lambda **kwargs: transport)
    node = backend.GazeboArduPilotBackend()
    node.get_clock = MagicMock()
    node.twist = SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=0, y=0, z=0), angular=SimpleNamespace(x=0, y=0, z=0)))
    return node, transport


def exchange(node, transport, frame_count, pose_time_s):
    sec = int(pose_time_s)
    node.pose = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=sec, nanosec=int(round((pose_time_s - sec) * 1e9)))),
        pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=0),
                             orientation=SimpleNamespace(w=1, x=0, y=0, z=0)))
    transport.receive.return_value = SimpleNamespace(pwm=[1000] * 4, frame_count=frame_count)
    node.tick()
    return transport.send.call_args.args[0]['timestamp']


def test_backend_sends_sitl_its_own_clock_not_gazebos(monkeypatch, tmp_path):
    node, transport = backend_node(monkeypatch, tmp_path)
    # SITL's first frame after Gazebo has run for 95 s (SITL started or
    # restarted against a running Gazebo).
    assert exchange(node, transport, 0, 95.0) == pytest.approx(START)
    assert exchange(node, transport, 1, 95.0) == pytest.approx(START)
    assert exchange(node, transport, 2, 95.01) == pytest.approx(START + 0.01)
    assert exchange(node, transport, 4000, 105.0) == pytest.approx(START + 10.0)
    # SITL restarted under a running backend.
    assert exchange(node, transport, 0, 400.0) == pytest.approx(START)
    starts = [c for c in node.get_logger().info.call_args_list
              if "SITL's clock starts" in c.args[0]]
    assert len(starts) == 2 and 'Gazebo t=400.000 s' in starts[1].args[0]


# ArduPilot 4.6.3 (the SITL images' Copter), as far as the sample rate goes:
#   AP_InertialSensor_SITL.h: the first IMU is registered at 1000 Hz;
#   AP_InertialSensor_SITL::timer_update: one sample per call once the clock
#     has passed the next sample time, so one per JSON time step (Gazebo's
#     100 Hz pose);
#   AP_InertialSensor_Backend::_update_sensor_rate: the estimate, every
#     second of samples, moves towards the observed rate, at most to half or
#     double and with weight 0.2 while sensors_converging() (clock under
#     HAL_INS_CONVERGANCE_MS = 30 s and disarmed), then at most +-5 % with
#     weight 0.02;
#   LowPassFilter2p (DigitalBiquadFilter): the 20 Hz INS_GYRO_FILTER built
#     for the estimate.
NOMINAL_HZ = 1000.0
DELIVERED_HZ = 100.0


def rate_estimate_at(first_stamp_s, armed_at_s, ins_start_s=2.0):
    """The IMU rate estimate when SITL arms, its clock starting at
    first_stamp_s (the first JSON timestamp)."""
    rate, start, count = NOMINAL_HZ, None, 0
    step = 1.0 / DELIVERED_HZ
    t = max(first_stamp_s, ins_start_s)
    while t < armed_at_s:
        if start is None:
            start, count = t, 0
        else:
            count += 1
            if t - start > 1.0:
                observed = count / (t - start)
                converging = t < 30.0
                weight, low, high = (0.8, 0.5, 2.0) if converging else (0.98, 0.95, 1.05)
                observed = min(max(observed, rate * low), rate * high)
                rate = weight * rate + (1 - weight) * observed
                start, count = t, 0
        t += step
    return rate


def gyro_delay_s(design_rate_hz, at_hz=2.2, cutoff_hz=20.0):
    """Phase delay of ArduPilot's 2-pole gyro filter built for
    design_rate_hz and fed DELIVERED_HZ samples."""
    ohm = math.tan(math.pi * cutoff_hz / design_rate_hz)
    c = 1 + 2 * math.cos(math.pi / 4) * ohm + ohm * ohm
    b0 = ohm * ohm / c
    a1 = 2 * (ohm * ohm - 1) / c
    a2 = (1 - 2 * math.cos(math.pi / 4) * ohm + ohm * ohm) / c
    z = cmath.exp(-1j * 2 * math.pi * at_hz / DELIVERED_HZ)  # z^-1
    response = b0 * (1 + 2 * z + z * z) / (1 + a1 * z + a2 * z * z)
    return -cmath.phase(response) / (2 * math.pi * at_hz)


def test_late_sitl_clock_keeps_the_nominal_imu_rate_and_a_slow_gyro():
    # SITL's clock at its first exchange in HIL: tens of seconds to minutes;
    # arming about 90 s later (the Pi's flight_ready).
    for first in (35.0, 60.0, 120.0):
        rate = rate_estimate_at(first, first + 90.0)
        assert rate > 0.85 * NOMINAL_HZ
        assert gyro_delay_s(rate) > 0.08


def test_sitl_clock_from_its_start_learns_the_delivered_rate_before_arming():
    rate = rate_estimate_at(START, 90.0)
    assert rate == pytest.approx(DELIVERED_HZ, rel=0.15)
    assert gyro_delay_s(rate) < 0.015


def sitl_launcher_args(tmp_path, **env):
    """The sim_vehicle.py arguments run_arducopter_sitl.sh passes, from a copy
    of the script run against a stand-in python3."""
    script = (REPO / 'applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh').read_text()
    assert 'cd /ardupilot\n' in script
    copy = tmp_path / 'run_arducopter_sitl.sh'
    copy.write_text(script.replace('cd /ardupilot\n', f'cd {tmp_path}\n'))
    stub = tmp_path / 'bin'
    stub.mkdir()
    (stub / 'python3').write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    (stub / 'python3').chmod(0o755)
    environment = {'PATH': f'{stub}:{os.environ["PATH"]}',
                   'TERRAIN_MANIFEST': str(tmp_path / 'none.json'), **env}
    out = subprocess.run(['bash', str(copy)], env=environment, check=True,
                         capture_output=True, text=True).stdout.split('\n')
    return out[1:]  # after Tools/autotest/sim_vehicle.py


def test_sitl_behind_a_physics_backend_does_not_wait_for_its_first_client(tmp_path):
    # SITL's default SERIAL0 is tcp:0:wait: it blocks until a MAVLink client
    # connects, and its clock then comes up at the wall time it waited, past
    # the 30 s whatever the backend's timestamps (all-local stack with
    # MAVROS held back 60 s: 936 Hz at arming with and without SitlClock).
    args = sitl_launcher_args(tmp_path, SIM_JSON_HOST='127.0.0.1')
    assert '--model' in args and 'JSON:127.0.0.1' in args
    assert '--sitl-instance-args=--serial0=tcp:0' in args
    assert not any('wait' in arg for arg in args)


def test_builtin_sitl_keeps_its_serial_defaults(tmp_path):
    args = sitl_launcher_args(tmp_path, ARDUCOPTER_SITL_MODEL='builtin')
    assert '--model' not in args
    assert not any('serial0' in arg for arg in args)

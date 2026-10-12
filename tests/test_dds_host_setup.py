"""Persistent DDS limits, including preserving larger operator settings."""
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from conftest import REPO, load_script


@pytest.fixture
def setup():
    return load_script(REPO / 'tools/configure_dds_buffers.py', 'dds_host_setup_test')


@pytest.mark.parametrize('initial', [
    (212992, 212992), (4194304, 4194304), (8388608, 212992), (8388608, 16777216),
])
def test_configuration_persists_applies_and_preserves_larger_limits(setup, monkeypatch, tmp_path, initial):
    values = dict(zip(setup.KEYS, initial))
    calls = []
    config = tmp_path / 'sysctl.d/99-cyberphysics-dds.conf'

    def sysctl(args, **kwargs):
        calls.append(args)
        if args[1] == '-n':
            return SimpleNamespace(stdout=str(values[args[2]]) + '\n')
        assert args == ['sysctl', '--load', str(config)]
        for line in Path(args[2]).read_text().splitlines():
            if not line.startswith('#'):
                key, value = line.split('=')
                values[key.strip()] = int(value)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(setup.subprocess, 'run', sysctl)
    expected = {key: max(value, 4194304) for key, value in zip(setup.KEYS, initial)}
    assert setup.configure(config) == expected
    first = config.read_text()
    assert setup.configure(config) == expected
    assert config.read_text() == first  # rerunning the installer is idempotent
    assert config.stat().st_mode & 0o777 == 0o644
    assert all(f'{key} = {value}\n' in first for key, value in expected.items())
    assert calls.count(['sysctl', '--load', str(config)]) == 2


@pytest.mark.parametrize('low_key', ['net.core.rmem_max', 'net.core.wmem_max'])
def test_read_only_preflight_rejects_either_small_limit(setup, monkeypatch, low_key):
    values = dict.fromkeys(setup.KEYS, 4194304)
    values[low_key] = 212992
    monkeypatch.setattr(setup, 'read_limits', lambda: values)
    with pytest.raises(RuntimeError, match=low_key):
        setup.check_limits()


def test_configuration_does_not_report_success_when_sysctl_fails(setup, monkeypatch, tmp_path):
    monkeypatch.setattr(setup, 'read_limits', lambda: dict.fromkeys(setup.KEYS, 212992))

    def fail(args, **kwargs):
        raise subprocess.CalledProcessError(1, args)

    monkeypatch.setattr(setup.subprocess, 'run', fail)
    with pytest.raises(subprocess.CalledProcessError):
        setup.configure(tmp_path / 'dds.conf')


@pytest.mark.parametrize('stack', ['racecarneo_hardware', 'racecar_turret_hardware',
                                  'turret_hardware', 'aerodrone_hardware',
                                  'boat_hardware', 'submarine_hardware'])
@pytest.mark.parametrize('ready', [False, True])
def test_start_refuses_to_launch_a_car_with_small_buffers(tmp_path, stack, ready):
    # No real sysctl or Docker commands: exercise start.sh's enforcement.
    python = tmp_path / 'python3'
    python.write_text('#!/bin/sh\n'
                      'test "$2" = --check || exit 99\n'
                      f'exit {0 if ready else 1}\n')
    python.chmod(0o755)
    docker = tmp_path / 'docker'
    marker = tmp_path / 'docker-called'
    docker.write_text('#!/bin/sh\ntouch "$DOCKER_MARKER"\n')
    docker.chmod(0o755)
    result = subprocess.run(['bash', str(REPO / 'start.sh'), stack, '-d'], capture_output=True,
                            env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}',
                                 'DOCKER_MARKER': str(marker)})
    assert (result.returncode == 0) == ready
    assert marker.exists() == ready

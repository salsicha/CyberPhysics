"""The OAK-1 node waits for its camera instead of exiting.

With no OAK-1 attached, depthai.Device raised "Cannot find any device with
given deviceInfo", megadepth_ros.py exited 1 and Docker restarted the
container in a loop (21 restarts in 10 minutes on the BlueOS Raspberry Pi 5).
"""
import sys
from unittest.mock import MagicMock, patch

# Imported before the fixture's patch.dict(sys.modules): numpy cannot be
# imported a second time in one process once the patch has dropped it.
import numpy  # noqa: F401
import pytest

from conftest import REPO, install_ros_stubs, load_script


@pytest.fixture(scope='module')
def megadepth():
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['depthai'] = MagicMock()
        yield load_script(REPO / 'applications/oak1/scripts/megadepth_ros.py',
                          'megadepth_device_wait')


class Logger:
    def __init__(self):
        self.lines = []

    def warning(self, text):
        self.lines.append(('warning', text))

    def info(self, text):
        self.lines.append(('info', text))


class Clock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


def flaky_device(failures):
    attempts = []

    def device(*args):
        attempts.append(args)
        if len(attempts) <= failures:
            raise RuntimeError('Cannot find any device with given deviceInfo')
        return 'device'
    return device, attempts


def test_waits_until_the_camera_is_plugged_in(megadepth):
    device, attempts = flaky_device(30)
    logger, clock = Logger(), Clock()
    with patch.object(megadepth.dai, 'Device', device):
        opened = megadepth.open_device('pipeline', 'auto', logger, 5.0,
                                       sleep=clock.sleep, clock=clock)
    assert opened == 'device'
    assert len(attempts) == 31 and attempts[0] == ('pipeline',)
    assert clock.sleeps == [5.0] * 30
    warnings = [text for level, text in logger.lines if level == 'warning']
    # Said once, then once a minute: at 0, 60 and 120 s of the 150 s wait.
    assert len(warnings) == 3
    assert 'Waiting for an OAK-1 on USB or the network' in warnings[0]
    assert 'Cannot find any device' in warnings[0]
    assert logger.lines[-1] == ('info', 'OAK-1 connected after 150 s')


def test_a_connected_camera_opens_at_once_and_quietly(megadepth):
    device, attempts = flaky_device(0)
    logger, clock = Logger(), Clock()
    with patch.object(megadepth.dai, 'Device', device):
        assert megadepth.open_device('pipeline', 'auto', logger,
                                     sleep=clock.sleep, clock=clock) == 'device'
    assert clock.sleeps == [] and logger.lines == []


def test_an_ip_camera_is_opened_by_its_address(megadepth):
    device, attempts = flaky_device(1)
    logger, clock = Logger(), Clock()
    with patch.object(megadepth.dai, 'Device', device), \
            patch.object(megadepth.dai, 'DeviceInfo', lambda ip: f'info:{ip}'):
        megadepth.open_device('pipeline', '169.254.1.222', logger,
                              sleep=clock.sleep, clock=clock)
    assert attempts == [('pipeline', 'info:169.254.1.222')] * 2
    assert 'at 169.254.1.222' in logger.lines[0][1]


def test_shutdown_ends_the_wait(megadepth):
    device, attempts = flaky_device(10 ** 6)
    logger, clock = Logger(), Clock()
    with patch.object(megadepth.dai, 'Device', device):
        opened = megadepth.open_device('pipeline', 'auto', logger,
                                       ok=lambda: len(attempts) < 3,
                                       sleep=clock.sleep, clock=clock)
    assert opened is None and len(attempts) == 3


def test_the_node_no_longer_exits_on_a_missing_camera():
    source = (REPO / 'applications/oak1/scripts/megadepth_ros.py').read_text()
    init = source[source.index('class MegaDepthNode'):]
    assert 'self.device = open_device(' in init
    assert 'dai.Device(pipeline)' not in init

"""Tello app: driver wrapper behaviour and compose wiring, without ROS."""
import importlib.util
import re
import sys
from unittest import mock

import pytest
import yaml

from conftest import REPO

APP = REPO / 'applications/tello'
STACK = yaml.safe_load((REPO / 'compositions/tello.yaml').read_text())['services']


class _Exit(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


class _ExternalShutdown(Exception):
    pass


def run_driver(monkeypatch, tello_node=None, spin=None):
    """Run tello_driver.main() with stub ROS modules; return (exit code, node)."""
    rclpy = mock.MagicMock()
    if spin is not None:
        rclpy.spin.side_effect = spin
    executors = mock.MagicMock(ExternalShutdownException=_ExternalShutdown)
    upstream = mock.MagicMock()
    if tello_node is not None:
        upstream.TelloNode.side_effect = tello_node
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', executors)
    monkeypatch.setitem(sys.modules, 'tello', mock.MagicMock())
    monkeypatch.setitem(sys.modules, 'tello.node', upstream)
    spec = importlib.util.spec_from_file_location('tello_driver', APP / 'tello_driver.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def fake_exit(code):
        raise _Exit(code)

    monkeypatch.setattr(module.os, '_exit', fake_exit)
    with pytest.raises(_Exit) as exited:
        module.main()
    return exited.value.code, rclpy.create_node.return_value


def test_missing_drone_is_one_error_line_and_a_retryable_exit(monkeypatch):
    error = Exception("Command 'command' was unsuccessful for 4 tries.")
    code, node = run_driver(monkeypatch, tello_node=error)
    assert code == 1
    message = node.get_logger.return_value.error.call_args.args[0]
    assert 'no connection to the drone' in message
    assert "Command 'command' was unsuccessful" in message


def test_network_unreachable_is_reported_the_same_way(monkeypatch):
    code, node = run_driver(
        monkeypatch, tello_node=OSError(101, 'Network is unreachable'))
    assert code == 1
    assert node.get_logger.return_value.error.called


@pytest.mark.parametrize('stop', [KeyboardInterrupt, _ExternalShutdown])
def test_shutdown_exits_zero_without_joining_driver_threads(monkeypatch, stop):
    code, node = run_driver(monkeypatch, spin=stop())
    assert code == 0
    assert not node.get_logger.return_value.error.called


def test_ctrl_c_while_connecting_is_a_clean_stop(monkeypatch):
    code, _ = run_driver(monkeypatch, tello_node=KeyboardInterrupt())
    assert code == 0


def test_compose_driver_and_teleop_stop_on_sigint():
    driver = STACK['tello']
    assert driver['network_mode'] == 'host'  # UDP 8889/8890/11111 to the drone
    assert driver['stop_signal'] == 'SIGINT'
    assert driver['command'].split()[:3] == ['ros2', 'launch', '/tello-ros2/tello.launch.py']
    teleop = STACK['tello_teleop']
    assert teleop['profiles'] == ['teleop']
    assert teleop['stop_signal'] == 'SIGINT'
    # The node binary itself is PID 1: `ros2 run` would not forward the signal
    assert teleop['command'].endswith('/lib/tello_control/tello_control')
    assert not teleop['command'].startswith('ros2 run')


def test_image_is_pinned_and_runs_the_headless_launch_file():
    dockerfile = (APP / 'Dockerfile').read_text()
    assert re.search(r'^ARG TELLO_ROS2_COMMIT=[0-9a-f]{40}$', dockerfile, re.M)
    assert 'refs/heads' not in dockerfile
    assert 'archive/${TELLO_ROS2_COMMIT}.zip' in dockerfile
    assert 'CMD ["ros2", "launch", "/tello-ros2/tello.launch.py"]' in dockerfile
    launch = (APP / 'tello.launch.py').read_text()
    assert "executable='/tello-ros2/tello_driver.py'" in launch
    assert 'respawn=True' in launch

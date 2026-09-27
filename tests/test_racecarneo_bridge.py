"""RACECAR Neo simulator bridge: steering sign, command safety, sensor noise.

Drives the bridge's real methods with ROS message modules stubbed out and a
mock RACECAR library, so no simulator or ROS graph is needed.
"""
import math
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs, load_script


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeTime((self.nanoseconds - other.nanoseconds) * 1e-9)

    def to_msg(self):
        return self.nanoseconds


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('ackermann_msgs', 'ackermann_msgs.msg', 'rcl_interfaces',
                     'rcl_interfaces.msg', 'tf2_ros'):
            sys.modules[name] = MagicMock()
        sys.modules['rcl_interfaces.msg'].SetParametersResult = lambda **kw: NS(**kw)
        yield load_script(REPO / 'applications/racecarneo/scripts/racecarneo_ros_bridge.py',
                          'racecarneo_bridge_review')


@pytest.fixture
def bridge(module):
    node = module.RacecarNeoBridge.__new__(module.RacecarNeoBridge)
    node.get_logger = MagicMock()
    node._rc = MagicMock()
    node._stop = False
    node._cmd_lock = threading.Lock()
    node._speed = node._steering = 0.0
    node._last_cmd_time = None
    node.command_timeout_s = 0.5
    node.manual_override = node.estop_engaged = False
    node._actual_speed_cmd = node._actual_steering_cmd = 0.0
    node._last_update_time = None
    # Instant actuator response so one update reaches the commanded value.
    node.throttle_time_constant_s = node.steering_time_constant_s = 1e-3
    node.max_speed, node.max_steering_angle, node.wheelbase = 0.35, 0.42, 0.33
    node.wheel_slip_std = node.encoder_quantization_m = 0.0
    node._x = node._y = node._yaw = 0.0
    node._last_speed_mps = node._last_yaw_rate = 0.0
    node.rng = np.random.default_rng(0)
    node.frame_prefix, node.map_frame, node.odom_frame, node.base_frame = (
        'racecarneo', 'map', 'odom', 'base_link')
    for flag in ('publish_camera', 'publish_depth', 'publish_lidar',
                 'publish_imu', 'publish_odom', 'publish_tf'):
        setattr(node, flag, False)
    node._publish_feedback = MagicMock()
    node._clock_s = 0.0
    node.get_clock = lambda: NS(now=lambda: FakeTime(node._clock_s))
    return node


def command(speed, steering):
    return NS(drive=NS(speed=speed, steering_angle=steering))


def step(bridge, dt=0.05):
    """Run one simulator update; the first call only initialises the clock."""
    if bridge._last_update_time is None:
        bridge._on_update()
    bridge._clock_s += dt
    bridge._on_update()
    return bridge._rc.drive.set_speed_angle.call_args.args


def test_ros_left_turn_is_library_left_and_odometry_agrees(bridge):
    bridge._cmd_callback(command(0.5, 0.5))  # ROS: steering_angle > 0 is a left turn
    speed, angle = step(bridge)
    assert speed == pytest.approx(0.5)
    # RACECAR library: -1 full left, +1 full right.
    assert angle == pytest.approx(-0.5)
    assert bridge._last_yaw_rate > 0.0  # odometry integrates it counter-clockwise


def test_watchdog_stops_car_when_commands_stop(bridge):
    bridge._cmd_callback(command(1.0, 0.2))
    assert step(bridge)[0] == pytest.approx(1.0)
    bridge._last_cmd_time -= bridge.command_timeout_s + 0.1  # stream went quiet
    speed, _angle = step(bridge)
    assert speed == pytest.approx(0.0)
    bridge.get_logger().warn.assert_called_once()
    assert step(bridge)[0] == pytest.approx(0.0)  # stale command is not resumed
    bridge._cmd_callback(command(0.4, 0.0))
    assert step(bridge)[0] == pytest.approx(0.4)


def test_no_command_yet_means_stopped(bridge):
    assert step(bridge)[0] == pytest.approx(0.0)


@pytest.mark.parametrize('speed,steering', [
    (math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0), (0.2, -math.inf)])
def test_non_finite_command_is_ignored(bridge, speed, steering):
    bridge._cmd_callback(command(speed, steering))
    assert bridge._last_cmd_time is None
    assert step(bridge) == (pytest.approx(0.0), pytest.approx(0.0))


def test_estop_parameter_takes_effect_while_running(bridge):
    bridge._cmd_callback(command(0.8, 0.3))
    assert step(bridge)[0] == pytest.approx(0.8)

    result = bridge._on_set_parameters([NS(name='estop_engaged', value=True)])
    assert result.successful
    bridge._cmd_callback(command(0.8, 0.3))
    assert step(bridge) == (pytest.approx(0.0), pytest.approx(0.0))

    bridge._on_set_parameters([NS(name='estop_engaged', value=False)])
    bridge._cmd_callback(command(0.8, 0.3))
    assert step(bridge)[0] == pytest.approx(0.8)


def test_lidar_noise_does_not_create_returns_from_no_return(bridge):
    samples_cm = np.zeros(720, dtype=np.float32)
    samples_cm[::4] = 250.0  # every fourth beam hits a wall at 2.5 m
    bridge._rc.lidar.get_samples.return_value = samples_cm
    bridge.lidar_noise_std_m = 0.015
    bridge.lidar_dropout_probability = 0.0
    bridge.lidar_pub = MagicMock()

    bridge._publish_scan(0)

    ranges = np.asarray(bridge.lidar_pub.publish.call_args.args[0].ranges)
    finite = ranges[np.isfinite(ranges)]
    assert finite.size == 180
    assert np.all(np.abs(finite - 2.5) < 0.1)


def test_depth_noise_leaves_invalid_pixels_invalid(bridge):
    depth_cm = np.zeros((24, 32), dtype=np.float32)
    depth_cm[:, 16:] = 300.0
    bridge._rc.camera.get_depth_image.return_value = depth_cm
    bridge.camera_dropout_probability = 0.0
    bridge.depth_noise_std_m = 0.015
    bridge.depth_dropout_probability = 0.0
    bridge.depth_pub = MagicMock()

    bridge._publish_depth(0)

    msg = bridge.depth_pub.publish.call_args.args[0]
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(24, 32)
    assert np.all(depth[:, :16] == 0.0)
    assert np.all(np.abs(depth[:, 16:] - 3.0) < 0.1)

"""RACECAR Neo simulator bridge: command units and safety, odometry, sensors.

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

MAX_SPEED = 0.35  # m/s at a full simulator command
MAX_STEER = 0.349  # rad at a full simulator command (RacecarSim: 20 deg)


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
    node.max_speed, node.max_steering_angle, node.wheelbase = MAX_SPEED, MAX_STEER, 0.33
    node.wheel_slip_std = node.encoder_quantization_m = 0.0
    node._x = node._y = node._yaw = 0.0
    node._encoder_residual_m = 0.0
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
    """ackermann_cmd in SI units: m/s and rad, positive left."""
    return NS(drive=NS(speed=speed, steering_angle=steering))


def ratio_command(speed_ratio, steering_ratio):
    """ackermann_cmd for a fraction of the bridge's speed and steering limits."""
    return command(speed_ratio * MAX_SPEED, steering_ratio * MAX_STEER)


def step(bridge, dt=0.05):
    """Run one simulator update; the first call only initialises the clock."""
    if bridge._last_update_time is None:
        bridge._on_update()
    bridge._clock_s += dt
    bridge._on_update()
    return bridge._rc.drive.set_speed_angle.call_args.args


def test_ros_left_turn_is_library_left_and_odometry_agrees(bridge):
    bridge._cmd_callback(ratio_command(0.5, 0.5))  # ROS: steering_angle > 0 is a left turn
    speed, angle = step(bridge)
    assert speed == pytest.approx(0.5)
    # RACECAR library: -1 full left, +1 full right.
    assert angle == pytest.approx(-0.5)
    assert bridge._last_yaw_rate > 0.0  # odometry integrates it counter-clockwise


def test_si_command_is_normalised_by_the_bridge_limits_and_clamped(bridge):
    bridge._cmd_callback(command(0.175, -0.1745))  # 0.175 m/s, 0.1745 rad right
    assert step(bridge) == (pytest.approx(0.5), pytest.approx(0.5, abs=1e-3))
    bridge._cmd_callback(command(-0.07, 0.0))
    assert step(bridge)[0] == pytest.approx(-0.2)
    # A consumer reading 1.0 as a ratio would floor it; as SI it is clamped.
    bridge._cmd_callback(command(1.0, 1.0))
    assert step(bridge) == (pytest.approx(1.0), pytest.approx(-1.0))


def test_feedback_reports_actuator_state_in_si(module, bridge):
    bridge.publish_actuator_feedback = True
    bridge.ackermann_feedback_pub = MagicMock()
    for name in ('manual_override_pub', 'estop_pub', 'battery_pub'):
        setattr(bridge, name, MagicMock())
    bridge.battery_nominal_voltage, bridge.battery_sag_per_speed = 11.1, 0.8

    module.RacecarNeoBridge._publish_feedback(bridge, 0, 0.5, -0.25)

    feedback = bridge.ackermann_feedback_pub.publish.call_args.args[0]
    assert feedback.drive.speed == pytest.approx(0.5 * MAX_SPEED)
    assert feedback.drive.steering_angle == pytest.approx(-0.25 * MAX_STEER)


def test_racecar_is_created_on_the_main_thread(bridge):
    fake_rc = MagicMock()

    def create_racecar(isSimulation):
        # RacecarSim's constructor calls signal.signal(SIGINT, ...), which
        # Python rejects off the main thread.
        if threading.current_thread() is not threading.main_thread():
            raise ValueError('signal only works in main thread of the main interpreter')
        return fake_rc

    with patch.dict(sys.modules, {'racecar_core': NS(create_racecar=create_racecar)}), \
            patch.object(sys, 'argv', list(sys.argv)):
        bridge.start()
        bridge._sim_thread.join(timeout=5.0)
    assert bridge._rc is fake_rc
    fake_rc.go.assert_called_once()  # go() itself runs on the simulator thread
    bridge.get_logger().error.assert_not_called()


@pytest.mark.parametrize('outcome, logged', [
    (None, 'warn'),  # the simulator sent its exit command
    (SystemExit(0), 'error'),  # the library's exit() on a protocol error
    (OSError('socket closed'), 'error'),
])
def test_bridge_shuts_down_when_the_simulator_session_ends(module, bridge, outcome, logged):
    # A bridge that outlives its simulator session would keep its topics
    # advertised with no data behind them.
    bridge._rc.go.side_effect = outcome
    with patch.object(module, 'rclpy') as rclpy:
        bridge._run_racecar()
    rclpy.try_shutdown.assert_called_once()
    getattr(bridge.get_logger(), logged).assert_called_once()


def test_stopping_bridge_does_not_report_the_session_end(module, bridge):
    bridge._stop = True  # node.stop() ran first, e.g. on SIGINT
    with patch.object(module, 'rclpy') as rclpy:
        bridge._run_racecar()
    rclpy.try_shutdown.assert_not_called()
    bridge.get_logger().warn.assert_not_called()


def test_watchdog_stops_car_when_commands_stop(bridge):
    bridge._cmd_callback(ratio_command(1.0, 0.2))
    assert step(bridge)[0] == pytest.approx(1.0)
    bridge._last_cmd_time -= bridge.command_timeout_s + 0.1  # stream went quiet
    speed, _angle = step(bridge)
    assert speed == pytest.approx(0.0)
    bridge.get_logger().warn.assert_called_once()
    assert step(bridge)[0] == pytest.approx(0.0)  # stale command is not resumed
    bridge._cmd_callback(ratio_command(0.4, 0.0))
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
    bridge._cmd_callback(ratio_command(0.8, 0.3))
    assert step(bridge)[0] == pytest.approx(0.8)

    result = bridge._on_set_parameters([NS(name='estop_engaged', value=True)])
    assert result.successful
    bridge._cmd_callback(ratio_command(0.8, 0.3))
    assert step(bridge) == (pytest.approx(0.0), pytest.approx(0.0))

    bridge._on_set_parameters([NS(name='estop_engaged', value=False)])
    bridge._cmd_callback(ratio_command(0.8, 0.3))
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


TICK = 0.002 + 1e-9  # one encoder tick, plus float rounding


def drive(bridge, speed_mps, seconds, hz=60.0, steering_rad=0.0):
    """Hold one command for `seconds` of simulator updates at `hz`."""
    bridge.command_timeout_s = 1e9  # the loop runs faster than real time
    bridge._cmd_callback(command(speed_mps, steering_rad))
    for _ in range(int(round(seconds * hz))):
        step(bridge, dt=1.0 / hz)


@pytest.mark.parametrize('speed', [0.05, 0.1, 0.25, -0.1])
def test_encoder_quantization_keeps_the_distance_travelled(bridge, speed):
    bridge.encoder_quantization_m = 0.002
    drive(bridge, speed, 10.0)
    # Per-step travel is below one tick at 0.05 m/s and 60 Hz; rounding the
    # integrated pose each step used to leave the car where it started.
    assert bridge._x == pytest.approx(speed * 10.0, abs=TICK)
    assert bridge._y == pytest.approx(0.0, abs=1e-9)


def test_encoder_quantization_off_axis(bridge):
    bridge.encoder_quantization_m = 0.002
    bridge._yaw = math.radians(10.0)
    drive(bridge, 0.25, 10.0)
    assert bridge._x == pytest.approx(2.5 * math.cos(math.radians(10.0)), abs=TICK)
    assert bridge._y == pytest.approx(2.5 * math.sin(math.radians(10.0)), abs=TICK)


def test_encoder_quantization_reports_whole_ticks(bridge):
    bridge.encoder_quantization_m = 0.002
    drive(bridge, 0.05, 1.0)
    ticks = math.hypot(bridge._x, bridge._y) / 0.002
    assert ticks == pytest.approx(round(ticks), abs=1e-6)
    assert abs(bridge._encoder_residual_m) < 0.002


def test_odometry_turn_matches_bicycle_model(bridge):
    bridge.encoder_quantization_m = 0.002
    steering = 0.2
    drive(bridge, 0.25, 4.0, steering_rad=steering)
    expected_yaw = 0.25 * 4.0 * math.tan(steering) / bridge.wheelbase
    assert bridge._yaw == pytest.approx(math.remainder(expected_yaw, 2 * math.pi), abs=0.01)


# RacecarSim v2.8 IMU: x right, y up, z forward; rates by the right-hand rule
# (left turn is +y); gravity added as -9.81 along up.
@pytest.mark.parametrize('sim_accel,sim_gyro,ros_accel,ros_gyro', [
    ((0.0, -9.81, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),  # at rest
    ((0.0, -9.81, 1.5), (0.0, 0.0, 0.0), (1.5, 0.0, 9.81), (0.0, 0.0, 0.0)),  # speeding up
    ((2.0, -9.81, 0.0), (0.0, 0.0, 0.0), (0.0, -2.0, 9.81), (0.0, 0.0, 0.0)),  # pushed right
    ((0.0, -9.81, 0.0), (0.0, 0.7, 0.0), (0.0, 0.0, 9.81), (0.0, 0.0, 0.7)),  # turning left
    ((0.0, -9.81, 0.0), (0.0, 0.0, 0.3), (0.0, 0.0, 9.81), (0.3, 0.0, 0.0)),  # rolling right
    ((0.0, -9.81, 0.0), (0.4, 0.0, 0.0), (0.0, 0.0, 9.81), (0.0, -0.4, 0.0)),  # nose up
])
def test_imu_is_converted_to_ros_flu(module, bridge, sim_accel, sim_gyro, ros_accel, ros_gyro):
    bridge._rc.physics.get_linear_acceleration.return_value = np.array(sim_accel, dtype=np.float32)
    bridge._rc.physics.get_angular_velocity.return_value = np.array(sim_gyro, dtype=np.float32)
    bridge.imu_accel_noise_std = bridge.imu_gyro_noise_std = 0.0
    bridge.imu_accel_bias = np.zeros(3, dtype=np.float32)
    bridge.imu_gyro_bias = np.zeros(3, dtype=np.float32)
    bridge.imu_pub = MagicMock()

    bridge._publish_imu(0)

    msg = bridge.imu_pub.publish.call_args.args[0]
    accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    assert accel == pytest.approx(ros_accel, abs=1e-5)
    assert gyro == pytest.approx(ros_gyro, abs=1e-5)


def test_imu_bias_is_applied_on_ros_axes(bridge):
    bridge._rc.physics.get_linear_acceleration.return_value = np.array([0.0, -9.81, 0.0])
    bridge._rc.physics.get_angular_velocity.return_value = np.zeros(3)
    bridge.imu_accel_noise_std = bridge.imu_gyro_noise_std = 0.0
    bridge.imu_accel_bias = np.array([0.02, -0.015, 0.0], dtype=np.float32)
    bridge.imu_gyro_bias = np.array([0.0, 0.0, 0.002], dtype=np.float32)
    bridge.imu_pub = MagicMock()

    bridge._publish_imu(0)

    msg = bridge.imu_pub.publish.call_args.args[0]
    assert msg.linear_acceleration.x == pytest.approx(0.02, abs=1e-6)
    assert msg.linear_acceleration.y == pytest.approx(-0.015, abs=1e-6)
    assert msg.angular_velocity.z == pytest.approx(0.002, abs=1e-6)


def test_camera_info_uses_unity_vertical_fov(bridge):
    bridge._rc.camera.get_width.return_value = 640
    bridge._rc.camera.get_height.return_value = 480
    bridge.camera_vertical_fov_degrees = 42.5
    bridge.camera_info_pub = MagicMock()

    bridge._publish_camera_info(0)

    k = bridge.camera_info_pub.publish.call_args.args[0].k
    fy = 240.0 / math.tan(math.radians(42.5) / 2.0)
    assert k[4] == pytest.approx(fy) and k[0] == pytest.approx(fy)
    assert (k[2], k[5]) == (320.0, 240.0)
    # Unity derives the horizontal FOV from the 4:3 aspect: ~54.8 deg, not 60.
    assert math.degrees(2.0 * math.atan(320.0 / k[0])) == pytest.approx(54.8, abs=0.1)

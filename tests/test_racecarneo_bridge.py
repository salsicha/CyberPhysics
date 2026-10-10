"""RACECAR Neo simulator bridge: command units and safety, speed control,
odometry, sensors.

Drives the bridge's real methods with ROS message modules stubbed out and a
mock RACECAR library, so no simulator or ROS graph is needed. Driving tests
use FakeSim, a RacecarSim stand-in whose throttle is motor torque on the
drive model the bridge was tuned on, with an IMU that reports its
acceleration and yaw rate.
"""
import math
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs, load_script

MAX_SPEED = 1.0  # m/s, bridge.yaml: ackermann_cmd speeds are clamped to it
MAX_STEER = 0.349  # rad at a full simulator command (RacecarSim: 20 deg)
SIM_WHEELBASE = 0.225


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeTime((self.nanoseconds - other.nanoseconds) * 1e-9)

    def to_msg(self):
        return self.nanoseconds


class FakeSim:
    """RacecarSim stand-in: throttle is torque, the IMU reports the result.

    dv/dt = gain * throttle - (drag + turn_drag * steering^2) * v (brake_gain
    while the throttle opposes the motion); exactly zero throttle is the
    simulator's wheel brake.
    `imu_bias` is added to the reported forward acceleration and `yaw_gain`
    scales the bicycle-model yaw rate (understeer < 1).
    """

    def __init__(self, gain=1.18, brake_gain=1.0, drag=0.85, turn_drag=0.15, brake_decel=1.0,
                 imu_bias=0.0, yaw_gain=1.0):
        self.gain, self.brake_gain, self.drag, self.turn_drag = gain, brake_gain, drag, turn_drag
        self.brake_decel, self.imu_bias, self.yaw_gain = brake_decel, imu_bias, yaw_gain
        self.v = self.accel = self.yaw_rate = 0.0
        self.throttle = self.angle = 0.0
        self.frame_s = 0.05
        self.drive = MagicMock()
        self.drive.set_speed_angle.side_effect = self._set_speed_angle
        self.physics = MagicMock()
        # Unity axes: x right, y up, z forward; gravity reads -9.81 on y.
        self.physics.get_linear_acceleration.side_effect = lambda: np.array(
            [0.0, -9.81, self.accel + self.imu_bias])
        self.physics.get_angular_velocity.side_effect = lambda: np.array(
            [0.0, self.yaw_rate, 0.0])

    def get_delta_time(self):
        return self.frame_s

    def _set_speed_angle(self, throttle, angle):
        # The RACECAR library asserts both are within [-1, 1].
        assert -1.0 <= throttle <= 1.0 and -1.0 <= angle <= 1.0
        self.throttle, self.angle = throttle, angle

    def advance(self, dt):
        """One frame of physics with the last throttle and steering."""
        v, u = self.v, self.throttle
        if u == 0.0:
            new_v = v - math.copysign(min(abs(v), self.brake_decel * dt), v)
        else:
            gain = self.brake_gain if v * u < 0.0 else self.gain
            drag = self.drag + self.turn_drag * self.angle ** 2
            new_v = v + (gain * u - drag * v) * dt
        self.accel = (new_v - v) / dt
        self.v = new_v
        wheel_angle = -self.angle * MAX_STEER  # library +1 is full right
        self.yaw_rate = self.yaw_gain * self.v * math.tan(wheel_angle) / SIM_WHEELBASE
        self.frame_s = dt


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
    # Instant command response so one update reaches the commanded value.
    node.throttle_time_constant_s = node.steering_time_constant_s = 1e-3
    node.max_speed, node.max_steering_angle, node.wheelbase = MAX_SPEED, MAX_STEER, SIM_WHEELBASE
    node.wheel_slip_std = node.encoder_quantization_m = 0.0
    node._x = node._y = node._yaw = 0.0
    node._encoder_residual_m = 0.0
    node._last_speed_mps = node._last_yaw_rate = 0.0
    node._throttle = node._sent_steering = 0.0
    node._speed_estimator = module.SpeedEstimator()
    node._speed_controller = module.SpeedController()
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


@pytest.fixture
def sim(bridge):
    bridge._rc = FakeSim()
    return bridge._rc


def command(speed, steering):
    """ackermann_cmd in SI units: m/s and rad, positive left."""
    return NS(drive=NS(speed=speed, steering_angle=steering))


def step(bridge, dt=0.05):
    """One simulator frame: physics, then the bridge's update.

    The first call only initialises the clock. Returns the (throttle, angle)
    the bridge sent to the RACECAR library.
    """
    if bridge._last_update_time is None:
        bridge._on_update()
    if isinstance(bridge._rc, FakeSim):
        bridge._rc.advance(dt)
    bridge._clock_s += dt
    bridge._on_update()
    return bridge._rc.drive.set_speed_angle.call_args.args


def drive(bridge, speed_mps, seconds, hz=20.0, steering_rad=0.0, log=None):
    """Hold one command for `seconds` of simulator frames at `hz`."""
    bridge.command_timeout_s = 1e9  # the loop runs faster than real time
    bridge._cmd_callback(command(speed_mps, steering_rad))
    for _ in range(round(seconds * hz)):
        out = step(bridge, dt=1.0 / hz)
        if log is not None:
            log.append((out[0], bridge._rc.v, bridge._last_speed_mps))


def test_ros_left_turn_is_library_left_and_odometry_agrees(bridge, sim):
    drive(bridge, 0.3, 2.0, steering_rad=0.5 * MAX_STEER)  # ROS: positive is a left turn
    _throttle, angle = sim.drive.set_speed_angle.call_args.args
    # RACECAR library: -1 full left, +1 full right.
    assert angle == pytest.approx(-0.5)
    assert sim.yaw_rate > 0.0
    assert bridge._last_yaw_rate > 0.0 and bridge._yaw > 0.0  # odometry turns left too


def test_si_command_is_a_speed_setpoint_and_a_steering_ratio(bridge):
    bridge._cmd_callback(command(0.175, -0.1745))  # 0.175 m/s, 0.1745 rad right
    assert (bridge._speed, bridge._steering) == (pytest.approx(0.175), pytest.approx(-0.5, abs=1e-3))
    bridge._cmd_callback(command(-0.07, 0.0))
    assert bridge._speed == pytest.approx(-0.07)
    # Clamped to the bridge limits rather than passed on as ratios.
    bridge._cmd_callback(command(3.0, 1.0))
    assert (bridge._speed, bridge._steering) == (pytest.approx(MAX_SPEED), pytest.approx(1.0))


def test_feedback_reports_estimated_speed_and_steering_in_si(module, bridge):
    bridge.publish_actuator_feedback = True
    bridge.ackermann_feedback_pub = MagicMock()
    for name in ('manual_override_pub', 'estop_pub', 'battery_pub'):
        setattr(bridge, name, MagicMock())
    bridge.battery_nominal_voltage, bridge.battery_sag_per_speed = 11.1, 0.8

    module.RacecarNeoBridge._publish_feedback(bridge, 0, 0.3, -0.25 * MAX_STEER)

    feedback = bridge.ackermann_feedback_pub.publish.call_args.args[0]
    assert feedback.drive.speed == pytest.approx(0.3)
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


def test_watchdog_stops_car_when_commands_stop(bridge, sim):
    log = []
    drive(bridge, 0.5, 4.0, log=log)
    assert sim.v == pytest.approx(0.5, rel=0.05)
    bridge.command_timeout_s = 0.5
    bridge._last_cmd_time -= bridge.command_timeout_s + 0.1  # stream went quiet
    throttles, speeds = [], []
    for _ in range(60):
        throttles.append(step(bridge)[0])
        speeds.append(sim.v)
    bridge.get_logger().warn.assert_called_once()
    assert throttles[0] < 0.0  # brakes against the motion
    assert throttles[-1] == 0.0  # then holds the simulator's brake
    assert min(speeds) >= 0.0  # and never reverses
    assert sim.v == pytest.approx(0.0, abs=1e-6)
    bridge._cmd_callback(command(0.3, 0.0))  # a new command resumes
    assert step(bridge)[0] > 0.0


def test_no_command_yet_means_stopped(bridge, sim):
    assert step(bridge)[0] == 0.0


@pytest.mark.parametrize('speed,steering', [
    (math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0), (0.2, -math.inf)])
def test_non_finite_command_is_ignored(bridge, sim, speed, steering):
    bridge._cmd_callback(command(speed, steering))
    assert bridge._last_cmd_time is None
    assert step(bridge) == (0.0, pytest.approx(0.0))


def test_estop_parameter_takes_effect_while_running(bridge, sim):
    drive(bridge, 0.4, 2.0, steering_rad=0.1)
    assert sim.throttle > 0.0

    result = bridge._on_set_parameters([NS(name='estop_engaged', value=True)])
    assert result.successful
    bridge._cmd_callback(command(0.4, 0.1))
    assert step(bridge) == (0.0, pytest.approx(0.0))  # motor cut: the sim's brake

    bridge._on_set_parameters([NS(name='estop_engaged', value=False)])
    bridge._cmd_callback(command(0.4, 0.1))
    assert step(bridge)[0] > 0.0


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


def drive_tracking_distance(bridge, speed_mps, seconds, hz=60.0):
    """drive(), returning the distance the speed estimate integrates to."""
    bridge.command_timeout_s = 1e9
    bridge._cmd_callback(command(speed_mps, 0.0))
    travelled = 0.0
    for _ in range(round(seconds * hz)):
        step(bridge, dt=1.0 / hz)
        travelled += bridge._last_speed_mps / hz
    return travelled


@pytest.mark.parametrize('speed', [0.05, 0.1, 0.25, -0.1])
def test_encoder_quantization_keeps_the_distance_travelled(bridge, sim, speed):
    bridge.encoder_quantization_m = 0.002
    travelled = drive_tracking_distance(bridge, speed, 10.0)
    assert abs(travelled) > abs(speed) * 8.0
    # Per-step travel is below one tick at 0.05 m/s and 60 Hz; rounding the
    # integrated pose each step used to leave the car where it started.
    assert bridge._x == pytest.approx(travelled, abs=TICK)
    assert bridge._y == pytest.approx(0.0, abs=1e-9)


def test_encoder_quantization_off_axis(bridge, sim):
    bridge.encoder_quantization_m = 0.002
    bridge._yaw = math.radians(10.0)
    travelled = drive_tracking_distance(bridge, 0.25, 10.0)
    assert bridge._x == pytest.approx(travelled * math.cos(math.radians(10.0)), abs=TICK)
    assert bridge._y == pytest.approx(travelled * math.sin(math.radians(10.0)), abs=TICK)


def test_encoder_quantization_reports_whole_ticks(bridge, sim):
    bridge.encoder_quantization_m = 0.002
    drive(bridge, 0.05, 1.0, hz=60.0)
    ticks = math.hypot(bridge._x, bridge._y) / 0.002
    assert ticks == pytest.approx(round(ticks), abs=1e-6)
    assert abs(bridge._encoder_residual_m) < 0.002


@pytest.mark.parametrize('yaw_gain', [1.0, 0.5])
def test_odometry_heading_is_the_simulator_yaw_rate(bridge, yaw_gain):
    # yaw_gain 0.5: the simulated car turns half as fast as the bicycle
    # model of the commanded steering says; odometry must follow the car.
    bridge._rc = FakeSim(yaw_gain=yaw_gain)
    imu_yaw = 0.0
    bridge.command_timeout_s = 1e9
    bridge._cmd_callback(command(0.25, 0.2))
    step(bridge)
    for _ in range(80):
        step(bridge)
        imu_yaw += bridge._rc.yaw_rate * 0.05
    assert bridge._yaw == pytest.approx(math.remainder(imu_yaw, 2 * math.pi), abs=1e-6)
    assert bridge._last_yaw_rate == pytest.approx(bridge._rc.yaw_rate)


def test_odometry_position_follows_the_car(bridge, sim):
    true_x = true_y = true_yaw = 0.0
    bridge.command_timeout_s = 1e9
    bridge._cmd_callback(command(0.3, 0.15))
    step(bridge)
    for _ in range(100):
        step(bridge)
        true_yaw += sim.yaw_rate * 0.05
        true_x += sim.v * 0.05 * math.cos(true_yaw - 0.5 * sim.yaw_rate * 0.05)
        true_y += sim.v * 0.05 * math.sin(true_yaw - 0.5 * sim.yaw_rate * 0.05)
    assert math.hypot(bridge._x - true_x, bridge._y - true_y) < 0.03 * math.hypot(true_x, true_y)


def test_odom_twist_is_the_estimated_speed_and_imu_yaw_rate(bridge, sim):
    bridge.odom_pub = MagicMock()
    drive(bridge, 0.3, 3.0, steering_rad=0.1)
    bridge._publish_odom(0)
    twist = bridge.odom_pub.publish.call_args.args[0].twist.twist
    assert twist.linear.x == pytest.approx(bridge._speed_estimator.speed)
    assert twist.linear.x == pytest.approx(sim.v, rel=0.05)
    assert twist.angular.z == pytest.approx(sim.yaw_rate)


def test_frame_time_is_the_simulator_delta_time(bridge, sim):
    # A simulator slower than real time (Unity caps frames at 0.1 s of
    # physics) integrates simulated, not wall-clock, seconds.
    bridge._on_update()
    sim.advance(0.1)
    bridge._clock_s += 0.5
    assert bridge._frame_time(FakeTime(bridge._clock_s)) == pytest.approx(0.1)
    sim.frame_s = math.nan  # unavailable: fall back to the ROS clock
    assert bridge._frame_time(FakeTime(bridge._clock_s)) == pytest.approx(0.5)


# ---------------------------------------------------------------- speed estimate


def run_estimator(estimator, accel, seconds, throttle, yaw_rate=0.0, dt=0.05):
    for _ in range(round(seconds / dt)):
        estimator.update(dt, accel, yaw_rate, throttle)
    return estimator.speed


def test_estimator_integrates_forward_acceleration(module):
    imu_only = module.SpeedEstimator(model_time_constant_s=0.0, braking_time_constant_s=0.0)
    assert run_estimator(imu_only, 0.4, 1.0, throttle=0.5) == pytest.approx(0.4)
    assert run_estimator(imu_only, -0.2, 1.0, throttle=-0.5) == pytest.approx(0.2)


def test_drive_model_bounds_imu_bias(module):
    # RacecarSim's IMU reads 0.02-0.05 m/s^2 high while the motor pushes.
    # Pure integration walks off; the model pull keeps the error at
    # bias * model_time_constant_s.
    car = FakeSim(imu_bias=0.04)
    estimators = {tau: module.SpeedEstimator(model_time_constant_s=tau, braking_time_constant_s=tau)
                  for tau in (0.0, 1.0)}
    throttle = 0.3
    for _ in range(400):  # 20 s at 20 Hz
        car.throttle = throttle
        car.advance(0.05)
        accel = car.accel + car.imu_bias
        for estimator in estimators.values():
            estimator.update(0.05, accel, 0.0, throttle)
    assert estimators[0.0].speed - car.v == pytest.approx(0.8, abs=0.01)
    assert abs(estimators[1.0].speed - car.v) < 0.05


def test_zero_velocity_update_at_rest(module):
    estimator = module.SpeedEstimator(model_time_constant_s=0.0, braking_time_constant_s=0.0)
    run_estimator(estimator, 0.5, 1.0, throttle=0.5)
    run_estimator(estimator, -1.2, 0.25, throttle=0.0)  # braking, not at rest yet
    assert estimator.speed == pytest.approx(0.2)
    run_estimator(estimator, 0.02, 0.25, throttle=0.0)  # still, but not long enough
    assert estimator.speed == pytest.approx(0.205)
    run_estimator(estimator, 0.02, 0.1, throttle=0.0)
    assert estimator.speed == 0.0 and estimator.model_speed == 0.0


@pytest.mark.parametrize('throttle,accel,yaw_rate', [
    (0.1, 0.0, 0.0),  # the motor is driving: cruising, not resting
    (0.0, -0.5, 0.0),  # still decelerating
    (0.0, 0.0, 0.3),  # turning
])
def test_no_zero_velocity_update_while_moving(module, throttle, accel, yaw_rate):
    estimator = module.SpeedEstimator(model_time_constant_s=0.0, braking_time_constant_s=0.0)
    estimator.speed = 0.3
    run_estimator(estimator, accel, 1.0, throttle, yaw_rate)
    assert estimator.speed == pytest.approx(0.3 + accel * 1.0)


# ---------------------------------------------------------------- speed controller


@pytest.mark.parametrize('setpoint', [0.05, 0.2, 0.5, -0.2])
def test_speed_controller_tracks_the_setpoint(bridge, sim, setpoint):
    log = []
    drive(bridge, setpoint, 10.0, log=log)
    assert sim.v == pytest.approx(setpoint, rel=0.03)
    # The integral only trims near the setpoint, so the rise does not overshoot.
    assert max(abs(entry[1]) for entry in log) <= abs(setpoint) + max(0.05 * abs(setpoint), 0.01)


def test_speed_integral_corrects_a_wrong_feedforward(bridge, sim):
    bridge._speed_controller.feedforward *= 0.6
    drive(bridge, 0.25, 15.0)
    assert sim.v == pytest.approx(0.25, rel=0.03)
    assert bridge._speed_controller.integral > 0.05


def test_speed_integral_only_integrates_near_the_setpoint(module):
    controller = module.SpeedController()
    controller.update(0.5, 0.0, 0.05)  # far below: proportional and feedforward only
    assert controller.integral == 0.0
    controller.update(0.5, 0.47, 0.05)
    assert controller.integral == pytest.approx(controller.ki * 0.03 * 0.05)


def test_estimate_does_not_see_a_car_that_differs_from_the_drive_model(bridge):
    # The limit of an IMU-plus-model estimate: in steady state the IMU reads
    # no acceleration, so the speed comes from the drive model. A car 30%
    # weaker than the model (RacecarSim at full steering lock is ~20%
    # slower) is driven correspondingly slower than commanded.
    bridge._rc = FakeSim(gain=0.8)
    drive(bridge, 0.25, 15.0)
    assert bridge._speed_estimator.speed == pytest.approx(0.25, rel=0.02)
    assert bridge._rc.v == pytest.approx(0.25 * 0.8 / 1.18, rel=0.05)


def test_brakes_with_reverse_throttle_then_holds_zero(bridge, sim):
    drive(bridge, 0.6, 5.0)
    log = []
    drive(bridge, 0.0, 3.0, log=log)
    throttles = [entry[0] for entry in log]
    assert throttles[0] == pytest.approx(-1.0)  # 2.0 * 0.6 m/s, clamped
    first_zero = throttles.index(0.0)
    assert all(t < 0.0 for t in throttles[:first_zero])
    assert all(t == 0.0 for t in throttles[first_zero:])  # simulator brake, held
    assert min(entry[1] for entry in log) >= 0.0  # never reversed
    assert sim.v == 0.0


def test_stop_command_skips_the_command_lag(bridge, sim):
    bridge.throttle_time_constant_s = 0.12
    drive(bridge, 0.4, 3.0)
    assert bridge._actual_speed_cmd == pytest.approx(0.4)
    bridge._cmd_callback(command(0.0, 0.0))
    assert step(bridge)[0] < 0.0  # brakes on the first update, no easing
    assert bridge._actual_speed_cmd == 0.0


def test_estimate_follows_the_drive_model_while_braking(module):
    # The simulator's IMU under-reads braking (here by 40%); while braking
    # the estimate leans on the drive model and reaches zero with the car.
    car = FakeSim()
    estimator = module.SpeedEstimator()
    slow_blend = module.SpeedEstimator(braking_time_constant_s=1.0)  # as while driving
    for throttle, seconds in ((0.35, 6.0), (-1.0, 0.3), (0.0, 1.0)):
        for _ in range(round(seconds / 0.05)):
            car.throttle = throttle
            car.advance(0.05)
            accel = car.accel * (0.6 if car.accel < 0.0 else 1.0)
            estimator.update(0.05, accel, 0.0, throttle)
            slow_blend.update(0.05, accel, 0.0, throttle)
        if throttle == -1.0:
            error = estimator.speed - car.v
            assert 0.0 < error < 0.6 * (slow_blend.speed - car.v)
    assert car.v == 0.0 and estimator.speed == 0.0


def test_brake_does_not_reverse_on_an_optimistic_estimate(module):
    controller = module.SpeedController()
    # The estimate still says 0.12 m/s after the car stopped: no reverse throttle.
    assert controller.update(0.0, 0.12, 0.05) == 0.0
    # Slowing for a lower forward setpoint brakes only while really moving.
    assert controller.update(0.05, 0.6, 0.05) < 0.0
    assert controller.update(0.02, 0.12, 0.05) == 0.0


def test_speed_controller_anti_windup(module):
    controller = module.SpeedController()
    # The car cannot reach 1 m/s (stuck at 0.2): the output saturates and
    # the integral must stop growing instead of winding up.
    for _ in range(200):
        assert controller.update(1.0, 0.2, 0.05) == pytest.approx(1.0)
    wound = controller.integral
    for _ in range(200):
        controller.update(1.0, 0.2, 0.05)
    assert controller.integral == pytest.approx(wound)
    assert abs(controller.integral) <= controller.integral_limit
    # Once the car is free the throttle backs off on the next update, not
    # after unwinding seconds of accumulated error.
    assert controller.update(0.3, 0.6, 0.05) < 0.0


def test_reverse_setpoint_drives_backwards(bridge, sim):
    drive(bridge, 0.3, 3.0)
    drive(bridge, -0.2, 8.0)
    assert sim.v == pytest.approx(-0.2, rel=0.05)


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

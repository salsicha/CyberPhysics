#!/usr/bin/env python3
"""RACECAR Turret simulation models, without Isaac Sim or ROS.

racecar_turret_isaac.py runs them every physics step; the tests check them
with plain numpy (tests/test_racecar_turret_isaac.py). Frames, topics and
the truth message are those of systems/racecar_turret/interfaces.md.

* Running gear: the Ackermann knuckle angles and the four wheel speeds of a
  bicycle (speed, steering) command, and the ESC and steering servo between
  /ackermann_cmd and the wheel drives (first-order lags, acceleration and
  slew limits, the 0.5 s command watchdog of racecarneo_ros_bridge.py).
* Sensor models: wheel odometry as the car computes it (rear wheel speed
  with a tyre radius error, encoder ticks and slip noise; yaw rate from the
  steering angle with an offset), so it drifts like the car's; the IMU
  (bias and noise of systems/racecarneo/config/sensor_models.yaml); the 2D
  lidar; pinhole camera intrinsics and the USD camera that renders them.
* The turret: two ST3215 servos behind turret_firmware (command latency,
  goal speed and acceleration, 4096-step encoder, 25 Hz position feedback,
  0.5 s hold watchdog).
* The scenario (systems/racecar_turret/scenarios): walls and obstacles, the
  car's collisions with them and with the people, whether a target is
  visible from the turret camera, and the /simulation/truth message.
"""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np

GRAVITY = 9.80665
STS_STEPS_PER_REV = 4096
STS_STEP_RAD = 2.0 * math.pi / STS_STEPS_PER_REV
TRUTH_TOPIC = "/simulation/truth"


# --- geometry ---------------------------------------------------------------

def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


def quat_wxyz_matrix(q):
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_quat_wxyz(rotation):
    """Unit quaternion (w, x, y, z) of a rotation matrix."""
    m = np.asarray(rotation, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    q = np.asarray(q)
    return q if q[0] >= 0.0 else -q


def yaw_of(rotation):
    return math.atan2(rotation[1][0], rotation[0][0])


# USD cameras look down their -z axis with +y up. This rotation turns that
# into a body frame looking down +x with +z up (x forward, y left, z up,
# like racecarneo/camera_link and turret/camera_link): (w, x, y, z).
USD_CAMERA_FROM_BODY_WXYZ = (0.5, 0.5, -0.5, -0.5)


# --- URDF frames ------------------------------------------------------------

class UrdfJoint:
    def __init__(self, name, kind, parent, child, origin, axis):
        self.name, self.kind, self.parent, self.child = name, kind, parent, child
        self.origin, self.axis = origin, axis


def urdf_joints(urdf_text):
    """{child link: UrdfJoint} of an expanded URDF."""
    root = ElementTree.fromstring(urdf_text)
    joints = {}
    for element in root.iter("joint"):
        origin = element.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        axis = element.find("axis")
        joint = UrdfJoint(
            element.get("name"), element.get("type"), element.find("parent").get("link"),
            element.find("child").get("link"), transform(xyz, rpy),
            np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None
            else np.array([1.0, 0.0, 0.0]))
        joints[joint.child] = joint
    return joints


def axis_rotation(axis, angle):
    """Rotation matrix by `angle` about a unit axis (Rodrigues)."""
    x, y, z = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def chain_transform(joints, ancestor, link, positions=None):
    """ancestor_T_link (4x4) with the movable joints at `positions` (by
    joint name). Raises ValueError when the chain does not reach ancestor
    or a movable joint has no position."""
    positions = positions or {}
    matrix = np.eye(4)
    while link != ancestor:
        joint = joints.get(link)
        if joint is None:
            raise ValueError(f"{link} does not descend from {ancestor}")
        step = joint.origin
        if joint.kind != "fixed":
            if joint.name not in positions:
                raise ValueError(f"{joint.name} between {ancestor} and {link} is {joint.kind}")
            motion = np.eye(4)
            if joint.kind == "prismatic":
                motion[:3, 3] = joint.axis * float(positions[joint.name])
            else:
                motion[:3, :3] = axis_rotation(joint.axis, float(positions[joint.name]))
            step = joint.origin @ motion
        matrix = step @ matrix
        link = joint.parent
    return matrix


def fixed_transform(joints, ancestor, link):
    """ancestor_T_link through fixed joints only (4x4). Raises ValueError
    when the chain crosses a movable joint or does not reach ancestor."""
    return chain_transform(joints, ancestor, link)


# --- running gear -----------------------------------------------------------

def ackermann_angles(steering, wheelbase, track):
    """(left, right) knuckle angles (rad, positive left) that steer both front
    wheels about the bicycle model's turn centre for `steering`."""
    if abs(steering) < 1e-9:
        return 0.0, 0.0
    radius = wheelbase / math.tan(steering)  # signed: positive turns left
    return (math.atan(wheelbase / (radius - track / 2.0)),
            math.atan(wheelbase / (radius + track / 2.0)))


def bicycle_steering(left, right, wheelbase, track):
    """The bicycle steering angle of measured knuckle angles (inverse of
    ackermann_angles, averaged over both wheels)."""
    estimates = []
    for angle, side in ((left, 1.0), (right, -1.0)):
        if abs(angle) < 1e-9:
            estimates.append(0.0)
            continue
        radius = wheelbase / math.tan(angle) + side * track / 2.0
        estimates.append(math.atan(wheelbase / radius))
    return 0.5 * (estimates[0] + estimates[1])


def wheel_rates(speed, steering, wheelbase, track, radius):
    """(front left, front right, rear left, rear right) wheel spin rates
    (rad/s, positive forward) of a 4x4 with open differentials rolling
    without slip: each wheel turns at its own path speed about the turn
    centre. `speed` is the bicycle model's (rear axle centre) speed."""
    if abs(steering) < 1e-9:
        rate = speed / radius
        return rate, rate, rate, rate
    turn = wheelbase / math.tan(steering)  # signed turn radius of the rear axle centre
    yaw_rate = speed / turn
    rear_left = yaw_rate * (turn - track / 2.0)
    rear_right = yaw_rate * (turn + track / 2.0)
    sign = math.copysign(1.0, speed) if speed else 0.0
    front_left = sign * abs(yaw_rate) * math.hypot(wheelbase, turn - track / 2.0)
    front_right = sign * abs(yaw_rate) * math.hypot(wheelbase, turn + track / 2.0)
    return tuple(v / radius for v in (front_left, front_right, rear_left, rear_right))


def _clamp(value, low, high):
    return max(low, min(high, value))


class DriveTrain:
    """ESC and steering servo between /ackermann_cmd and the wheels.

    ``command`` takes /ackermann_cmd (SI: m/s, rad positive left), clamped to
    the vehicle's limits; non-finite commands are ignored. ``update`` moves
    the speed toward the command through the ESC's first-order lag
    (speed_time_constant_s) within its acceleration and braking limits, and
    the steering through the servo's lag within its slew rate. Without a
    command for command_timeout_s (simulation time) the speed command
    becomes zero and the steering holds, as racecarneo_ros_bridge.py does.
    Lags and the timeout are those of systems/racecarneo/config (bridge.yaml,
    sensor_models.yaml); the slew rate is a 20 kg-cm servo's 0.16 s/60 deg.
    """

    def __init__(self, wheelbase=0.33, track=0.30, wheel_radius=0.055, max_speed=1.0,
                 max_steering=0.42, speed_time_constant_s=0.12, max_accel=2.0,
                 max_decel=3.0, steering_time_constant_s=0.09, max_steering_rate=6.5,
                 command_timeout_s=0.5):
        self.wheelbase, self.track, self.wheel_radius = wheelbase, track, wheel_radius
        self.max_speed, self.max_steering = max_speed, max_steering
        self.speed_time_constant_s = max(1e-3, speed_time_constant_s)
        self.max_accel, self.max_decel = max_accel, max_decel
        self.steering_time_constant_s = max(1e-3, steering_time_constant_s)
        self.max_steering_rate = max_steering_rate
        self.command_timeout_s = command_timeout_s
        self.target_speed = self.target_steering = 0.0
        self.speed = self.steering = 0.0
        self.last_command_t = None
        self.timed_out = False

    def command(self, t, speed, steering):
        if not (math.isfinite(speed) and math.isfinite(steering)):
            return False
        self.target_speed = _clamp(float(speed), -self.max_speed, self.max_speed)
        self.target_steering = _clamp(float(steering), -self.max_steering, self.max_steering)
        self.last_command_t = float(t)
        self.timed_out = False
        return True

    def stop(self):
        """Zero speed command (e-stop): brake, steering held."""
        self.target_speed = 0.0

    def update(self, t, dt):
        """Advance by dt; returns (speed, steering) applied to the wheels."""
        if self.last_command_t is None or t - self.last_command_t > self.command_timeout_s:
            self.timed_out = self.last_command_t is not None
            self.target_speed = 0.0
        if dt <= 0.0:
            return self.speed, self.steering
        if self.target_speed == 0.0:
            step = -self.speed  # brake at once (within max_decel), as the bridge
        else:
            step = (self.target_speed - self.speed) * min(1.0, dt / self.speed_time_constant_s)
        speeding_up = self.speed * step > 0.0 or (self.speed == 0.0 and step != 0.0)
        limit = (self.max_accel if speeding_up else self.max_decel) * dt
        self.speed += _clamp(step, -limit, limit)
        step = (self.target_steering - self.steering) * min(1.0, dt / self.steering_time_constant_s)
        limit = self.max_steering_rate * dt
        self.steering += _clamp(step, -limit, limit)
        return self.speed, self.steering

    def knuckle_angles(self):
        return ackermann_angles(self.steering, self.wheelbase, self.track)

    def wheel_rates(self):
        return wheel_rates(self.speed, self.steering, self.wheelbase, self.track,
                           self.wheel_radius)


# --- sensor models ----------------------------------------------------------

class WheelOdometry:
    """Odometry as the car computes it from the VESC and the steering servo
    (vesc_to_odom): speed = rear wheel spin x nominal tyre radius, in whole
    encoder ticks, with slip noise; yaw rate = speed x tan(steering) / L.
    The tyre's true radius differs from the nominal by radius_error and the
    steering trim by steering_offset_rad, so the pose drifts as a real
    car's does (it is never the simulator's ground truth)."""

    def __init__(self, wheelbase=0.33, wheel_radius=0.055, radius_error=0.02,
                 steering_offset_rad=0.004, slip_std=0.025, quantization_m=0.002, rng=None):
        self.wheelbase, self.wheel_radius = wheelbase, wheel_radius
        self.radius_error, self.steering_offset_rad = radius_error, steering_offset_rad
        self.slip_std, self.quantization_m = slip_std, quantization_m
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0
        self._residual_m = 0.0

    def update(self, dt, rear_wheel_rate, steering):
        """rear_wheel_rate: mean rear wheel spin (rad/s); steering: the
        bicycle steering angle the servo is at (rad)."""
        if dt <= 0.0:
            return self.pose()
        speed = rear_wheel_rate * self.wheel_radius * (1.0 + self.radius_error)
        if self.slip_std > 0.0:
            speed *= 1.0 + float(self.rng.normal(0.0, self.slip_std))
        distance = self._ticks(speed * dt)
        speed = distance / dt
        yaw_rate = speed * math.tan(steering + self.steering_offset_rad) / self.wheelbase
        heading = self.yaw + 0.5 * yaw_rate * dt
        self.x += distance * math.cos(heading)
        self.y += distance * math.sin(heading)
        self.yaw = wrap(self.yaw + yaw_rate * dt)
        self.speed, self.yaw_rate = speed, yaw_rate
        return self.pose()

    def _ticks(self, distance):
        """Travel in whole encoder ticks, the remainder carried over."""
        if self.quantization_m <= 0.0:
            return distance
        self._residual_m += distance
        measured = math.trunc(self._residual_m / self.quantization_m) * self.quantization_m
        self._residual_m -= measured
        return measured

    def pose(self):
        return self.x, self.y, self.yaw, self.speed, self.yaw_rate


class ImuModel:
    """A body-mounted IMU (REP-145: an accelerometer at rest reads +g up).

    The specific force is the body's acceleration (finite differences of the
    simulator's velocity, low-passed with filter_time_constant_s: contact
    forces make the raw difference ring) minus gravity, in the body frame;
    the rate gyro reads the body's angular velocity. Bias and white noise
    are added on the body axes."""

    def __init__(self, accel_noise_std=0.08, gyro_noise_std=0.015,
                 accel_bias=(0.02, -0.015, 0.0), gyro_bias=(0.001, -0.001, 0.002),
                 filter_time_constant_s=0.015, rng=None):
        self.accel_noise_std, self.gyro_noise_std = accel_noise_std, gyro_noise_std
        self.accel_bias = np.asarray(accel_bias, dtype=np.float64)
        self.gyro_bias = np.asarray(gyro_bias, dtype=np.float64)
        self.filter_time_constant_s = filter_time_constant_s
        self.rng = rng if rng is not None else np.random.default_rng(1)
        self._velocity = None
        self._acceleration = np.zeros(3)

    def update(self, dt, velocity_world):
        """Track the body's acceleration; call every physics step."""
        velocity = np.asarray(velocity_world, dtype=np.float64)
        if self._velocity is not None and dt > 0.0:
            raw = (velocity - self._velocity) / dt
            alpha = 1.0 - math.exp(-dt / self.filter_time_constant_s) if self.filter_time_constant_s > 0 else 1.0
            self._acceleration += alpha * (raw - self._acceleration)
        self._velocity = velocity

    def ideal(self, rotation_world_body, angular_velocity_world):
        """(specific force, angular rate) on the body axes, without errors."""
        rotation = np.asarray(rotation_world_body, dtype=np.float64)
        accel = rotation.T @ (self._acceleration + np.array([0.0, 0.0, GRAVITY]))
        gyro = rotation.T @ np.asarray(angular_velocity_world, dtype=np.float64)
        return accel, gyro

    def measure(self, rotation_world_body, angular_velocity_world):
        accel, gyro = self.ideal(rotation_world_body, angular_velocity_world)
        accel = accel + self.accel_bias + self.rng.normal(0.0, self.accel_noise_std, 3)
        gyro = gyro + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise_std, 3)
        return accel, gyro


def scan_directions(count):
    """Unit ray directions of a 360 degree 2D scan in the lidar frame,
    LaserScan order: angle_min = -pi, counter-clockwise by 2 pi / count."""
    angles = -math.pi + np.arange(count) * (2.0 * math.pi / count)
    return np.stack([np.cos(angles), np.sin(angles), np.zeros(count)], axis=1)


def scan_ranges(depths, range_min, range_max, noise_std=0.0, dropout=0.0, rng=None):
    """LaserScan ranges of raycast depths: no return (non-finite, below
    range_min, or at or beyond range_max, where a raycast that hit nothing
    may report its maximum) is inf, as rplidar_ros reports it; range noise
    only on returns; dropped returns become inf."""
    ranges = np.asarray(depths, dtype=np.float64).copy()
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
    if rng is not None and noise_std > 0.0:
        ranges = np.where(valid, ranges + rng.normal(0.0, noise_std, ranges.shape), ranges)
        valid &= ranges >= range_min
    if rng is not None and dropout > 0.0:
        valid &= rng.random(ranges.shape) >= dropout
    return np.where(valid, ranges, np.inf).astype(np.float32)


def depth_measurement(depth, noise_std=0.0, dropout=0.0, rng=None, max_range=None):
    """32FC1 depth (metres) of a rendered distance_to_image_plane: no
    surface (inf, beyond max_range) is NaN; noise only on valid pixels;
    dropped pixels are NaN."""
    depth = np.asarray(depth, dtype=np.float32).copy()
    valid = np.isfinite(depth) & (depth > 0.0)
    if max_range is not None:
        valid &= depth <= max_range
    if rng is not None and noise_std > 0.0:
        depth = np.where(valid, depth + rng.normal(0.0, noise_std, depth.shape).astype(np.float32), depth)
    if rng is not None and dropout > 0.0:
        valid &= rng.random(depth.shape) >= dropout
    return np.ascontiguousarray(np.where(valid, depth, np.nan).astype(np.float32))


def pinhole(width, height, fx, fy=None):
    """(fx, fy, cx, cy) of a rendered camera in ROS's convention, pixel
    centres at integers: the optical axis meets the image at
    ((W - 1) / 2, (H - 1) / 2) (applications/isaac/README.md)."""
    fy = fx if fy is None else fy
    return float(fx), float(fy), (width - 1) / 2.0, (height - 1) / 2.0


def fx_of_hfov(width, hfov_deg):
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def hfov_of_fx(width, fx):
    return math.degrees(2.0 * math.atan((width / 2.0) / fx))


def usd_apertures(width, height, fx, horizontal_aperture_mm=20.955):
    """(focal length, horizontal aperture, vertical aperture) in mm of a USD
    camera rendering width x height with square pixels and focal length fx
    pixels: RTX maps the horizontal aperture onto the image width."""
    focal = fx * horizontal_aperture_mm / width
    return focal, horizontal_aperture_mm, horizontal_aperture_mm * height / width


def camera_info_values(width, height, fx, fy, cx, cy):
    return {
        "width": int(width), "height": int(height), "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


# --- turret -----------------------------------------------------------------

class ServoAxis:
    """One ST3215 in position mode: it accelerates toward its goal at the
    goal acceleration, at most at the goal speed (turret_firmware writes
    both with every SET), brakes to stop on the goal and holds within half
    an encoder step of it."""

    def __init__(self, max_velocity, max_acceleration, lower, upper, position=0.0):
        self.max_velocity, self.max_acceleration = max_velocity, max_acceleration
        self.lower, self.upper = lower, upper
        self.position = float(position)
        self.velocity = 0.0

    def update(self, dt, goal):
        goal = _clamp(goal, self.lower, self.upper)
        error = goal - self.position
        if abs(error) <= 0.5 * STS_STEP_RAD and abs(self.velocity) <= self.max_acceleration * dt:
            self.velocity = 0.0
            return self.position
        stopping = math.copysign(math.sqrt(2.0 * self.max_acceleration * abs(error)), error)
        desired = _clamp(stopping, -self.max_velocity, self.max_velocity)
        change = self.max_acceleration * dt
        self.velocity += _clamp(desired - self.velocity, -change, change)
        position = self.position + self.velocity * dt
        if (goal - position) * error < 0.0:  # would pass the goal: stop on it
            position, self.velocity = goal, 0.0
        self.position = _clamp(position, self.lower, self.upper)
        if self.position in (self.lower, self.upper):
            self.velocity = 0.0
        return self.position


class TurretActuator:
    """The two ST3215 servos behind turret_firmware and turret_arduino_bridge.

    A /turret/joint_commands position reaches the servos latency_s later
    (USB serial, firmware loop, servo bus). The firmware writes the goal with
    its goal speed (2000 steps/s, 3.07 rad/s) and acceleration (50 x 100
    steps/s^2, 7.67 rad/s^2) (turret_config.h), holds the measured position
    when no SET arrives for watchdog_s or while the emergency stop is
    engaged, and reports positions in whole encoder steps every 40 ms, each
    published feedback_delay_s after it was read and stamped with the
    reading's time.
    """

    def __init__(self, pan_limits=(-1.5708, 1.5708), tilt_limits=(-0.65, 0.75),
                 max_velocity=2000 * STS_STEP_RAD, max_acceleration=5000 * STS_STEP_RAD,
                 latency_s=0.03, watchdog_s=0.5, feedback_period_s=0.04, feedback_delay_s=0.01):
        self.limits = (tuple(pan_limits), tuple(tilt_limits))
        self.axes = (ServoAxis(max_velocity, max_acceleration, *pan_limits),
                     ServoAxis(max_velocity, max_acceleration, *tilt_limits))
        self.latency_s, self.watchdog_s = latency_s, watchdog_s
        self.feedback_period_s, self.feedback_delay_s = feedback_period_s, feedback_delay_s
        self.queue = deque()
        self.goal = (0.0, 0.0)
        self.last_set_t = None
        self.emergency_stop = False
        self._holding = False
        self.history = deque()
        self._next_feedback_t = 0.0

    def command(self, t, pan, tilt):
        """Queue a joint command received at simulation time t."""
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            return False
        pan = _clamp(float(pan), *self.limits[0])
        tilt = _clamp(float(tilt), *self.limits[1])
        self.queue.append((float(t) + self.latency_s, pan, tilt))
        return True

    def update(self, t, dt):
        """Servo goals executed by t; returns the (pan, tilt) positions the
        servos drive to over dt."""
        while self.queue and self.queue[0][0] <= t:
            applied_t, pan, tilt = self.queue.popleft()
            if not self.emergency_stop:
                self.goal, self.last_set_t = (pan, tilt), applied_t
        holding = (self.emergency_stop or self.last_set_t is None
                   or t - self.last_set_t > self.watchdog_s)
        if holding and not self._holding:
            # The firmware writes the measured position as the goal, once.
            self.goal = (self.axes[0].position, self.axes[1].position)
        if self.emergency_stop:
            self.queue.clear()
        self._holding = holding
        return tuple(axis.update(dt, goal) for axis, goal in zip(self.axes, self.goal))

    def record(self, t, pan, tilt):
        """Record the joints' true positions at simulation time t (from the
        physics) for the position feedback."""
        self.history.append((float(t), float(pan), float(tilt)))
        while len(self.history) > 2 and t - self.history[1][0] > 1.0:
            self.history.popleft()

    def feedback(self, t):
        """(stamp, pan, tilt) of a position report due at t, or None: the
        reading taken feedback_delay_s earlier, in whole encoder steps,
        stamped with the time it was read (the tracker fits the actuation
        delay from these stamps)."""
        if t + 1e-9 < self._next_feedback_t or not self.history:
            return None
        self._next_feedback_t = max(self._next_feedback_t + self.feedback_period_s, t)
        read_t = t - self.feedback_delay_s
        sample = self.history[0]
        for entry in self.history:
            if entry[0] > read_t + 1e-9:
                break
            sample = entry
        return (sample[0], quantize_steps(sample[1]), quantize_steps(sample[2]))


def quantize_steps(angle):
    return round(angle / STS_STEP_RAD) * STS_STEP_RAD


# --- scenario ---------------------------------------------------------------

class Obstacle:
    """A box (centre, size, yaw about its centre) or a vertical cylinder,
    standing on the floor (z = 0)."""

    def __init__(self, name, kind, center, size=None, radius=None, height=None, yaw=0.0,
                 color=(0.5, 0.5, 0.5)):
        self.name, self.kind = name, kind
        self.center = np.asarray(center[:2], dtype=np.float64)
        self.yaw = float(yaw)
        self.color = tuple(float(c) for c in color)
        if kind == "box":
            self.size = np.asarray(size, dtype=np.float64)
            self.height = float(self.size[2])
        elif kind == "cylinder":
            self.radius, self.height = float(radius), float(height)
        else:
            raise ValueError(f"{name}: unknown obstacle type {kind!r}")

    @classmethod
    def from_json(cls, entry):
        return cls(entry["name"], entry["type"], entry["center"], size=entry.get("size"),
                   radius=entry.get("radius"), height=entry.get("height"),
                   yaw=math.radians(float(entry.get("yaw_deg", 0.0))),
                   color=entry.get("color", (0.5, 0.5, 0.5)))

    def _local(self, points):
        """Points (N, 2+) in the box frame (x, y about the centre)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        delta = np.asarray(points, dtype=np.float64)[..., :2] - self.center
        return np.stack([c * delta[..., 0] + s * delta[..., 1],
                         -s * delta[..., 0] + c * delta[..., 1]], axis=-1)

    def blocks(self, start, end):
        """Whether the segment start-end (3D) passes through the obstacle."""
        start = np.asarray(start, dtype=np.float64)
        end = np.asarray(end, dtype=np.float64)
        low, high = 0.0, 1.0
        dz = end[2] - start[2]
        low, high = _slab(start[2], dz, 0.0, self.height, low, high)
        if low > high:
            return False
        if self.kind == "box":
            a, b = self._local(np.stack([start, end]))
            d = b - a
            for axis in (0, 1):
                half = self.size[axis] / 2.0
                low, high = _slab(a[axis], d[axis], -half, half, low, high)
                if low > high:
                    return False
            return True
        a = start[:2] - self.center
        d = end[:2] - start[:2]
        qa = float(d @ d)
        qb = 2.0 * float(a @ d)
        qc = float(a @ a) - self.radius ** 2
        if qa < 1e-12:
            return qc <= 0.0
        disc = qb * qb - 4.0 * qa * qc
        if disc < 0.0:
            return False
        root = math.sqrt(disc)
        t0, t1 = (-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)
        return max(low, t0) <= min(high, t1)

    def touches_rectangle(self, center, yaw, length, width):
        """Whether a rectangle on the floor (the car's footprint) overlaps
        the obstacle's footprint."""
        if self.kind == "cylinder":
            return circle_touches_rectangle(self.center, self.radius, center, yaw, length, width)
        return rectangles_overlap(self.center, self.yaw, self.size[0], self.size[1],
                                  np.asarray(center[:2], dtype=np.float64), yaw, length, width)


def _slab(origin, direction, low_bound, high_bound, low, high):
    """Clip the segment parameter interval [low, high] to one slab."""
    if abs(direction) < 1e-12:
        return (low, high) if low_bound <= origin <= high_bound else (1.0, 0.0)
    t0 = (low_bound - origin) / direction
    t1 = (high_bound - origin) / direction
    if t0 > t1:
        t0, t1 = t1, t0
    return max(low, t0), min(high, t1)


def _corners(center, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    half = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * np.array([length / 2.0, width / 2.0])
    return np.asarray(center[:2]) + half @ np.array([[c, s], [-s, c]])


def rectangles_overlap(center_a, yaw_a, length_a, width_a, center_b, yaw_b, length_b, width_b):
    """Separating-axis test of two rectangles on the floor."""
    a = _corners(center_a, yaw_a, length_a, width_a)
    b = _corners(center_b, yaw_b, length_b, width_b)
    for yaw in (yaw_a, yaw_b):
        for axis in (np.array([math.cos(yaw), math.sin(yaw)]),
                     np.array([-math.sin(yaw), math.cos(yaw)])):
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def circle_touches_rectangle(circle_center, radius, center, yaw, length, width):
    return rectangle_distance(circle_center, center, yaw, length, width) <= radius


def arena_walls(arena):
    """The arena's four walls as boxes: arena = {"min_xy", "max_xy",
    "wall_height_m", "wall_thickness_m"}; the walls stand outside the
    rectangle."""
    (x0, y0), (x1, y1) = arena["min_xy"], arena["max_xy"]
    height = float(arena.get("wall_height_m", 1.0))
    thick = float(arena.get("wall_thickness_m", 0.2))
    color = arena.get("wall_color", (0.62, 0.6, 0.56))
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    width, depth = x1 - x0 + 2 * thick, y1 - y0 + 2 * thick
    return [
        Obstacle("arena_wall_south", "box", (cx, y0 - thick / 2), (width, thick, height), color=color),
        Obstacle("arena_wall_north", "box", (cx, y1 + thick / 2), (width, thick, height), color=color),
        Obstacle("arena_wall_west", "box", (x0 - thick / 2, cy), (thick, depth, height), color=color),
        Obstacle("arena_wall_east", "box", (x1 + thick / 2, cy), (thick, depth, height), color=color),
    ]


def load_scenario(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def scenario_obstacles(scenario):
    """Walls and obstacles of a scenario document."""
    return arena_walls(scenario["arena"]) + [Obstacle.from_json(entry)
                                             for entry in scenario.get("obstacles", [])]


def people_config(scenario, models):
    """The tracking_targets.json-style document of the scenario's people
    (tracking_targets.resolve_tracks input): `models` is the models section
    of the file the scenario's people.models_from names."""
    people = scenario["people"]
    return {
        "sample_spacing_m": float(people.get("sample_spacing_m", 0.1)),
        "models": {key: models[key] for key in {entry["model"] for entry in people["targets"]}},
        "targets": people["targets"],
    }


class Spawn:
    """The odom frame: the car's spawn pose in the world. Odometry starts
    there, and SLAM's map starts equal to it."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.origin = np.array([self.x, self.y, 0.0])

    def point(self, world):
        return self.rotation.T @ (np.asarray(world, dtype=np.float64) - self.origin)

    def vector(self, world):
        return self.rotation.T @ np.asarray(world, dtype=np.float64)

    def yaw_of(self, world_yaw):
        return wrap(world_yaw - self.yaw)


def rectangle_distance(point, center, yaw, length, width):
    """Distance from a point on the floor to a rectangle (0 inside it)."""
    c, s = math.cos(yaw), math.sin(yaw)
    delta = np.asarray(point[:2], dtype=np.float64) - np.asarray(center[:2], dtype=np.float64)
    local = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
    nearest = np.clip(local, [-length / 2.0, -width / 2.0], [length / 2.0, width / 2.0])
    return float(np.linalg.norm(local - nearest))


class Walkers:
    """People walking their scripted tracks (tracking_targets.Tracks) who
    give way to the car, as pedestrians do: a person's clock along their
    track stops while the next step would bring them within `clearance` of
    the car's footprint and closer to it. Walking away from the car or past
    it is never held up, so a person never walks into the car, and the car
    (which stops for people in its way) and a person never wait for each
    other."""

    def __init__(self, tracks, radius, clearance=0.25):
        self.tracks = tracks
        self.radius, self.clearance = float(radius), float(clearance)
        self.times = [0.0] * len(tracks.tracks)
        self.waited = [0.0] * len(tracks.tracks)

    def poses(self):
        return [track.pose(t) for track, t in zip(self.tracks.tracks, self.times)]

    def step(self, dt, car_center, car_yaw, length, width):
        """Advance every person by dt unless they give way; returns their poses."""
        for index, track in enumerate(self.tracks.tracks):
            now = track.pose(self.times[index])
            ahead = track.pose(self.times[index] + dt)
            distance = rectangle_distance((ahead.east, ahead.north), car_center, car_yaw, length, width)
            if (distance < self.radius + self.clearance
                    and distance < rectangle_distance((now.east, now.north), car_center, car_yaw,
                                                      length, width)):
                self.waited[index] += dt
                continue
            self.times[index] += dt
        return self.poses()


class CollisionCounter:
    """Collisions of the car's footprint with obstacles and people: a new
    contact (a name entering the set of touched ones) counts once."""

    def __init__(self, length, width):
        self.length, self.width = length, width
        self.touching = set()
        self.count = 0
        self.events = []

    def update(self, t, center, yaw, obstacles, people=()):
        touched = {o.name for o in obstacles if o.touches_rectangle(center, yaw, self.length, self.width)}
        touched |= {name for name, xy, radius in people
                    if circle_touches_rectangle(xy, radius, center, yaw, self.length, self.width)}
        for name in sorted(touched - self.touching):
            self.count += 1
            self.events.append((float(t), name))
        self.touching = touched
        return self.count


BODY_SAMPLES = (0.15, 0.35, 0.5, 0.7, 0.9)


def visible_fraction(eye, base, height, occluders, samples=BODY_SAMPLES):
    """Fraction of points up a standing target's body (base on the floor,
    `height` tall) with a clear line of sight from `eye`."""
    base = np.asarray(base, dtype=np.float64)
    clear = 0
    for fraction in samples:
        point = base + np.array([0.0, 0.0, fraction * height])
        if not any(o.blocks(eye, point) for o in occluders):
            clear += 1
    return clear / len(samples)


def project(point, position, forward, up, fx, fy, cx, cy):
    """Pixel (u, v) of a world point in a camera at `position` looking along
    `forward` with `up` (REP-103 optical: x right, y down), or None behind."""
    forward = np.asarray(forward, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    right = np.cross(forward, up)
    delta = np.asarray(point, dtype=np.float64) - np.asarray(position, dtype=np.float64)
    depth = float(delta @ forward)
    if depth <= 1e-6:
        return None
    return cx + fx * float(delta @ right) / depth, cy - fy * float(delta @ up) / depth


def aim_error_deg(position, forward, point):
    """Angle between a camera's optical axis and its line of sight to a point."""
    forward = np.asarray(forward, dtype=np.float64)
    sight = np.asarray(point, dtype=np.float64) - np.asarray(position, dtype=np.float64)
    norm = np.linalg.norm(forward) * np.linalg.norm(sight)
    if norm <= 0.0:
        return float("nan")
    return math.degrees(math.acos(_clamp(float(forward @ sight) / norm, -1.0, 1.0)))


def _rounded(values, digits=4):
    return [round(float(v), digits) for v in values]


def truth_message(stamp, vehicle, camera, targets, collisions):
    """/simulation/truth content (systems/racecar_turret/interfaces.md), all
    in the odom frame. vehicle: (x, y, yaw, speed); camera: (position,
    forward, up) of turret/camera_link; targets: dicts with name, class,
    position (on the floor), centroid, visible and in_view."""
    x, y, yaw, speed = vehicle
    position, forward, up = camera
    return {
        "stamp": round(float(stamp), 6),
        "vehicle": {"x": round(float(x), 4), "y": round(float(y), 4),
                    "yaw": round(float(yaw), 5), "speed_mps": round(float(speed), 4)},
        "turret_camera": {"position": _rounded(position), "forward": _rounded(forward, 5),
                          "up": _rounded(up, 5)},
        "targets": [{
            "name": target["name"], "class": target["class"],
            "position": _rounded(target["position"], 3),
            "centroid": _rounded(target["centroid"], 3),
            "visible": bool(target["visible"]),
            "in_view": bool(target["in_view"]),
        } for target in targets],
        "collisions": int(collisions),
    }

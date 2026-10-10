#!/usr/bin/env python3
import math
import os
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, Header
from tf2_ros import TransformBroadcaster


GRAVITY_MPS2 = 9.81


def sim_imu_to_ros(accel, gyro):
    """Convert RacecarSim v2.8 (protocol 1) IMU vectors to ROS FLU.

    The simulator reports body vectors on Unity axes: x right, y up,
    z forward (racecar library physics.py). Angular velocity is the negated
    Unity (left-handed) rigidbody rate, i.e. right-hand-rule rates about
    those axes: a left turn is +y. ROS FLU is x forward, y left, z up, so
    (x, y, z)_ros = (z, -x, y)_sim for both vectors.

    The simulator adds gravity as Vector3.down * 9.81 (a car at rest reads
    -9.81 along up), the opposite sign of a real accelerometer, which reads
    the +9.81 reaction force (REP-145). The sim does not report attitude,
    so the correction assumes a level car: add 2 g along body up. The sim
    differentiates body-frame velocity, so centripetal acceleration is
    missing from its lateral axis.

    v2.8 also takes roll/pitch rates from world rather than body axes, so
    only the yaw rate is exact once the car turns away from its spawn
    heading. RacecarSim v2.9+ reports REP-103 axes directly; drop this
    conversion if the image is moved to that simulator.
    """
    ax, ay, az = (float(v) for v in accel)
    gx, gy, gz = (float(v) for v in gyro)
    accel_ros = np.array([az, -ax, ay + 2.0 * GRAVITY_MPS2])
    gyro_ros = np.array([gz, -gx, gy])
    return accel_ros, gyro_ros


class RacecarNeoBridge(Node):
    def __init__(self) -> None:
        super().__init__('racecarneo_bridge')
        self.declare_parameter('frame_prefix', 'racecarneo')
        self.declare_parameter('max_speed', 0.35)
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('publish_lidar', True)
        self.declare_parameter('publish_imu', True)
        # Unity's Camera.fieldOfView is vertical; RacecarSim sets 42.5 deg.
        self.declare_parameter('camera_vertical_fov_degrees', 42.5)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('wheelbase', 0.225)
        # RacecarSim v2.8 steers the front wheels angle * 20 deg.
        self.declare_parameter('max_steering_angle', 0.349)
        self.declare_parameter('sensor_random_seed', 101)
        self.declare_parameter('camera_dropout_probability', 0.0)
        self.declare_parameter('camera_motion_blur_pixels', 0)
        self.declare_parameter('depth_noise_std_m', 0.015)
        self.declare_parameter('depth_dropout_probability', 0.01)
        self.declare_parameter('lidar_noise_std_m', 0.015)
        self.declare_parameter('lidar_dropout_probability', 0.005)
        self.declare_parameter('imu_accel_noise_std', 0.08)
        self.declare_parameter('imu_gyro_noise_std', 0.015)
        self.declare_parameter('imu_accel_bias', [0.02, -0.015, 0.0])
        self.declare_parameter('imu_gyro_bias', [0.001, -0.001, 0.002])
        self.declare_parameter('wheel_slip_std', 0.025)
        self.declare_parameter('encoder_quantization_m', 0.002)
        self.declare_parameter('throttle_time_constant_s', 0.12)
        self.declare_parameter('steering_time_constant_s', 0.09)
        self.declare_parameter('battery_nominal_voltage', 11.1)
        self.declare_parameter('battery_sag_per_speed', 0.8)
        self.declare_parameter('manual_override', False)
        self.declare_parameter('estop_engaged', False)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('publish_actuator_feedback', True)

        self.frame_prefix = self.get_parameter('frame_prefix').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.publish_camera = bool(self.get_parameter('publish_camera').value)
        self.publish_depth = bool(self.get_parameter('publish_depth').value)
        self.publish_lidar = bool(self.get_parameter('publish_lidar').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.camera_vertical_fov_degrees = float(self.get_parameter('camera_vertical_fov_degrees').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        seed = int(self.get_parameter('sensor_random_seed').value)
        self.rng = np.random.default_rng(seed)
        self.camera_dropout_probability = float(self.get_parameter('camera_dropout_probability').value)
        self.camera_motion_blur_pixels = int(self.get_parameter('camera_motion_blur_pixels').value)
        self.depth_noise_std_m = float(self.get_parameter('depth_noise_std_m').value)
        self.depth_dropout_probability = float(self.get_parameter('depth_dropout_probability').value)
        self.lidar_noise_std_m = float(self.get_parameter('lidar_noise_std_m').value)
        self.lidar_dropout_probability = float(self.get_parameter('lidar_dropout_probability').value)
        self.imu_accel_noise_std = float(self.get_parameter('imu_accel_noise_std').value)
        self.imu_gyro_noise_std = float(self.get_parameter('imu_gyro_noise_std').value)
        self.imu_accel_bias = np.asarray(self.get_parameter('imu_accel_bias').value, dtype=np.float32)
        self.imu_gyro_bias = np.asarray(self.get_parameter('imu_gyro_bias').value, dtype=np.float32)
        self.wheel_slip_std = float(self.get_parameter('wheel_slip_std').value)
        self.encoder_quantization_m = float(self.get_parameter('encoder_quantization_m').value)
        self.throttle_time_constant_s = max(1e-3, float(self.get_parameter('throttle_time_constant_s').value))
        self.steering_time_constant_s = max(1e-3, float(self.get_parameter('steering_time_constant_s').value))
        self.battery_nominal_voltage = float(self.get_parameter('battery_nominal_voltage').value)
        self.battery_sag_per_speed = float(self.get_parameter('battery_sag_per_speed').value)
        self.manual_override = bool(self.get_parameter('manual_override').value)
        self.estop_engaged = bool(self.get_parameter('estop_engaged').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.publish_actuator_feedback = bool(self.get_parameter('publish_actuator_feedback').value)
        # E-stop, manual override, and the watchdog timeout can change at
        # runtime, e.g. `ros2 param set /racecarneo_bridge estop_engaged true`.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        # Travel not yet reported as a whole encoder tick.
        self._encoder_residual_m = 0.0
        self._last_update_time = None
        self._last_speed_mps = 0.0
        self._last_yaw_rate = 0.0

        self._cmd_lock = threading.Lock()
        self._speed = 0.0
        self._steering = 0.0
        self._last_cmd_time: Optional[float] = None
        self._actual_speed_cmd = 0.0
        self._actual_steering_cmd = 0.0
        self._rc = None
        self._stop = False
        self._sim_thread: Optional[threading.Thread] = None

        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 5)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_rect_raw', 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 5)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', 5)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 5)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.ackermann_feedback_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_feedback', 10)
        self.manual_override_pub = self.create_publisher(Bool, 'racecarneo/manual_override', 5)
        self.estop_pub = self.create_publisher(Bool, 'racecarneo/estop', 5)
        self.battery_pub = self.create_publisher(Float32, 'racecarneo/battery_voltage', 5)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(AckermannDriveStamped, 'ackermann_cmd', self._cmd_callback, 10)

    def _cmd_callback(self, msg: AckermannDriveStamped) -> None:
        # ackermann_cmd is SI (m/s, rad, positive left); the simulator takes
        # ratios of its own limits.
        speed = float(msg.drive.speed)
        steering = float(msg.drive.steering_angle)
        # min/max would turn NaN into a full-scale command; drop it and let
        # the watchdog stop the car if valid commands do not resume.
        if not (math.isfinite(speed) and math.isfinite(steering)):
            self.get_logger().warn('Ignoring non-finite ackermann_cmd', throttle_duration_sec=1.0)
            return
        speed = speed / self.max_speed if self.max_speed > 0.0 else 0.0
        steering = steering / self.max_steering_angle if self.max_steering_angle > 0.0 else 0.0
        speed = max(-1.0, min(1.0, speed))
        steering = max(-1.0, min(1.0, steering))
        with self._cmd_lock:
            self._speed = speed
            self._steering = steering
            self._last_cmd_time = time.monotonic()

    def _on_set_parameters(self, params) -> SetParametersResult:
        with self._cmd_lock:
            for param in params:
                if param.name == 'estop_engaged':
                    self.estop_engaged = bool(param.value)
                    self.get_logger().warn(f'E-stop {"engaged" if self.estop_engaged else "released"}')
                elif param.name == 'manual_override':
                    self.manual_override = bool(param.value)
                elif param.name == 'command_timeout_s':
                    self.command_timeout_s = float(param.value)
        return SetParametersResult(successful=True)

    def start(self) -> None:
        # The library's constructor installs a SIGINT handler, which Python
        # only allows on the main thread, so only go() runs in the background.
        sys.argv = [sys.argv[0], '-s', '-h']
        import racecar_core
        self._rc = racecar_core.create_racecar(isSimulation=True)
        self._rc.set_start_update(self._on_start, self._on_update, self._on_update_slow)
        self._sim_thread = threading.Thread(target=self._run_racecar, daemon=True)
        self._sim_thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._rc is not None:
            try:
                self._rc.drive.stop()
            except Exception:
                pass

    def _run_racecar(self) -> None:
        # go() connects, then waits for the simulator to enter user-program
        # mode. RacecarSim v2.8.3 only does that on its START input (Enter),
        # which run_headless_sim.sh presses; the library's racecar_go packet
        # is not handled by this simulator version.
        try:
            self._rc.go()
        except (Exception, SystemExit) as exc:
            # The library reports protocol errors (no free car, timeout,
            # version mismatch) by printing them and calling exit().
            self.get_logger().error(f'RACECAR Neo simulator bridge stopped: {exc!r}')
            rclpy.try_shutdown()
            return
        if not self._stop:
            # The simulator sent its exit command (it is shutting down).
            self.get_logger().warn('RACECAR Neo simulator closed the connection; shutting down')
            rclpy.try_shutdown()

    def _on_start(self) -> None:
        self.get_logger().info('RACECAR Neo simulator connected; publishing ROS topics')
        try:
            self._rc.drive.set_max_speed(self.max_speed)
        except Exception as exc:
            self.get_logger().warn(f'Unable to set simulator max speed: {exc}')

    def _on_update_slow(self) -> None:
        pass

    def _on_update(self) -> None:
        if self._stop:
            try:
                self._rc.drive.stop()
            finally:
                return

        now = self.get_clock().now()
        stamp = now.to_msg()
        speed_cmd, steering_cmd = self._update_vehicle_state(now)
        # ROS steering (and this bridge's odometry) is positive to the left;
        # the RACECAR library's angle is -1 full left, +1 full right.
        self._rc.drive.set_speed_angle(speed_cmd, -steering_cmd)

        if self.publish_camera:
            self._publish_color(stamp)
            self._publish_camera_info(stamp)
        if self.publish_depth:
            self._publish_depth(stamp)
        if self.publish_lidar:
            self._publish_scan(stamp)
        if self.publish_imu:
            self._publish_imu(stamp)
        if self.publish_odom:
            self._publish_odom(stamp)
        if self.publish_tf:
            self._publish_tf(stamp)
        self._publish_feedback(stamp, speed_cmd, steering_cmd)

    def _header(self, stamp, frame: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = self._frame(frame)
        return header

    def _frame(self, frame: str) -> str:
        if frame in (self.map_frame, self.odom_frame, self.base_frame):
            return frame
        return f'{self.frame_prefix}/{frame}'

    def _update_vehicle_state(self, now):
        if self._last_update_time is None:
            self._last_update_time = now
            return self._actual_speed_cmd, self._actual_steering_cmd
        dt = max(0.0, (now - self._last_update_time).nanoseconds * 1e-9)
        self._last_update_time = now
        if dt <= 0.0:
            return self._actual_speed_cmd, self._actual_steering_cmd

        with self._cmd_lock:
            # Watchdog: never keep driving on a command stream that stopped.
            timed_out = (self._last_cmd_time is None
                         or time.monotonic() - self._last_cmd_time > self.command_timeout_s)
            if timed_out and self._speed != 0.0:
                self.get_logger().warn(
                    f'No ackermann_cmd for {self.command_timeout_s:.2f}s; stopping')
                self._speed = 0.0
            target_speed = self._speed
            target_steering = self._steering
            manual_override = self.manual_override
            estop_engaged = self.estop_engaged
        if manual_override or estop_engaged:
            target_speed = 0.0
            if estop_engaged:
                target_steering = 0.0

        throttle_alpha = min(1.0, dt / self.throttle_time_constant_s)
        steering_alpha = min(1.0, dt / self.steering_time_constant_s)
        self._actual_speed_cmd += (target_speed - self._actual_speed_cmd) * throttle_alpha
        self._actual_steering_cmd += (target_steering - self._actual_steering_cmd) * steering_alpha

        # Open-loop model: RacecarSim v2.8 turns the speed ratio into motor
        # torque and reports no wheel speed, so odometry assumes a full
        # command reaches max_speed. It is not measured simulator motion.
        speed = float(self._actual_speed_cmd) * self.max_speed
        speed *= 1.0 + float(self.rng.normal(0.0, self.wheel_slip_std))
        steering_angle = float(self._actual_steering_cmd) * self.max_steering_angle
        curvature = math.tan(steering_angle) / self.wheelbase if abs(self.wheelbase) > 1e-6 else 0.0
        distance = self._encoder_distance(speed * dt)
        yaw = self._yaw + distance * curvature
        self._yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self._x += distance * math.cos(self._yaw)
        self._y += distance * math.sin(self._yaw)
        self._last_speed_mps = speed
        self._last_yaw_rate = speed * curvature
        return self._actual_speed_cmd, self._actual_steering_cmd

    def _encoder_distance(self, distance: float) -> float:
        """Report travel in whole encoder ticks and carry the remainder.

        Rounding the integrated pose instead discards every sub-tick step,
        so the car never moves at low speed and drifts short at any speed.
        """
        if self.encoder_quantization_m <= 0.0:
            return distance
        self._encoder_residual_m += distance
        ticks = math.trunc(self._encoder_residual_m / self.encoder_quantization_m)
        measured = ticks * self.encoder_quantization_m
        self._encoder_residual_m -= measured
        return measured

    def _yaw_quaternion(self):
        half = 0.5 * self._yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _publish_odom(self, stamp) -> None:
        qx, qy, qz, qw = self._yaw_quaternion()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self._last_speed_mps
        msg.twist.twist.angular.z = self._last_yaw_rate
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.2
        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[35] = 0.2
        self.odom_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        # Sensor extrinsics (base_link -> racecarneo/*) come from
        # robot_state_publisher and systems/racecarneo/urdf, shared with the
        # hardware stack; the bridge only owns odom -> base_link.
        qx, qy, qz, qw = self._yaw_quaternion()
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = self._x
        odom_to_base.transform.translation.y = self._y
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(odom_to_base)

    def _drop(self, probability: float) -> bool:
        return probability > 0.0 and float(self.rng.random()) < probability

    def _publish_color(self, stamp) -> None:
        if self._drop(self.camera_dropout_probability):
            return
        image = self._rc.camera.get_color_image_no_copy()
        if image is None:
            return
        image = np.ascontiguousarray(image)
        if self.camera_motion_blur_pixels > 1:
            blurred = image.astype(np.float32)
            for offset in range(1, self.camera_motion_blur_pixels):
                blurred += np.roll(image, shift=offset, axis=1).astype(np.float32)
            image = np.clip(blurred / float(self.camera_motion_blur_pixels), 0, 255).astype(np.uint8)
        msg = Image()
        msg.header = self._header(stamp, 'camera_color_optical_frame')
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * image.shape[2])
        msg.data = image.tobytes()
        self.color_pub.publish(msg)

    def _publish_depth(self, stamp) -> None:
        if self._drop(self.camera_dropout_probability):
            return
        depth_cm = self._rc.camera.get_depth_image()
        if depth_cm is None:
            return
        depth_m = depth_cm.astype(np.float32) / 100.0
        if self.depth_noise_std_m > 0.0:
            # 0 means no return; noise must not turn it into a phantom surface.
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            noise = self.rng.normal(0.0, self.depth_noise_std_m, size=depth_m.shape).astype(np.float32)
            depth_m = np.where(valid, depth_m + noise, depth_m)
        if self.depth_dropout_probability > 0.0:
            mask = self.rng.random(depth_m.shape) < self.depth_dropout_probability
            depth_m[mask] = np.nan
        depth_m = np.ascontiguousarray(depth_m)
        msg = Image()
        msg.header = self._header(stamp, 'camera_depth_optical_frame')
        msg.height = int(depth_m.shape[0])
        msg.width = int(depth_m.shape[1])
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = int(depth_m.shape[1] * 4)
        msg.data = depth_m.tobytes()
        self.depth_pub.publish(msg)

    def _publish_camera_info(self, stamp) -> None:
        width = int(self._rc.camera.get_width())
        height = int(self._rc.camera.get_height())
        fy = (height / 2.0) / math.tan(math.radians(self.camera_vertical_fov_degrees) / 2.0)
        fx = fy  # square pixels; Unity derives the horizontal FOV from the aspect
        cx = width / 2.0
        cy = height / 2.0
        msg = CameraInfo()
        msg.header = self._header(stamp, 'camera_color_optical_frame')
        msg.width = width
        msg.height = height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(msg)

    def _publish_scan(self, stamp) -> None:
        raw_cm = self._rc.lidar.get_samples()
        if raw_cm is None:
            return
        raw_m = np.asarray(raw_cm, dtype=np.float32) / 100.0
        if self.lidar_noise_std_m > 0.0:
            # 0 means no return; noise must not turn it into a phantom obstacle.
            valid = np.isfinite(raw_m) & (raw_m > 0.0)
            noise = self.rng.normal(0.0, self.lidar_noise_std_m, size=raw_m.shape).astype(np.float32)
            raw_m = np.where(valid, raw_m + noise, raw_m)
        if self.lidar_dropout_probability > 0.0:
            raw_m[self.rng.random(raw_m.shape) < self.lidar_dropout_probability] = np.inf
        count = raw_m.shape[0]
        msg = LaserScan()
        msg.header = self._header(stamp, 'lidar_link')
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / float(count - 1)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.02
        msg.range_max = 12.0
        ranges = []
        clockwise_increment = 2.0 * math.pi / float(count)
        for i in range(count):
            angle = msg.angle_min + i * msg.angle_increment
            source_index = int(round((-angle % (2.0 * math.pi)) / clockwise_increment)) % count
            value = float(raw_m[source_index])
            ranges.append(value if math.isfinite(value) and value > 0.0 else math.inf)
        msg.ranges = ranges
        self.lidar_pub.publish(msg)

    def _publish_imu(self, stamp) -> None:
        msg = Imu()
        msg.header = self._header(stamp, 'imu_link')
        accel, gyro = sim_imu_to_ros(self._rc.physics.get_linear_acceleration(),
                                     self._rc.physics.get_angular_velocity())
        # Bias and noise are specified on the ROS (FLU) axes.
        accel = accel + self.imu_accel_bias + self.rng.normal(0.0, self.imu_accel_noise_std, size=3)
        gyro = gyro + self.imu_gyro_bias + self.rng.normal(0.0, self.imu_gyro_noise_std, size=3)
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration_covariance = [self.imu_accel_noise_std ** 2, 0.0, 0.0, 0.0, self.imu_accel_noise_std ** 2, 0.0, 0.0, 0.0, self.imu_accel_noise_std ** 2]
        msg.angular_velocity_covariance = [self.imu_gyro_noise_std ** 2, 0.0, 0.0, 0.0, self.imu_gyro_noise_std ** 2, 0.0, 0.0, 0.0, self.imu_gyro_noise_std ** 2]
        msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def _publish_feedback(self, stamp, speed_cmd, steering_cmd) -> None:
        if self.publish_actuator_feedback:
            msg = AckermannDriveStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.base_frame
            # SI like ackermann_cmd: the actuator state, not the raw ratio.
            msg.drive.speed = float(speed_cmd) * self.max_speed
            msg.drive.steering_angle = float(steering_cmd) * self.max_steering_angle
            self.ackermann_feedback_pub.publish(msg)
        manual = Bool()
        manual.data = bool(self.manual_override)
        self.manual_override_pub.publish(manual)
        estop = Bool()
        estop.data = bool(self.estop_engaged)
        self.estop_pub.publish(estop)
        battery = Float32()
        battery.data = float(self.battery_nominal_voltage - self.battery_sag_per_speed * abs(self._last_speed_mps) / max(self.max_speed, 1e-6))
        self.battery_pub.publish(battery)


def main() -> None:
    os.environ.setdefault('RACECAR_SIM_IP', '127.0.0.1')
    rclpy.init()
    node = RacecarNeoBridge()
    node.start()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.001)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM, or the simulator thread ending the session
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

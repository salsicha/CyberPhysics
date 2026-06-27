#!/usr/bin/env python3
import math
import os
import socket
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, Header
from tf2_ros import TransformBroadcaster


class RacecarNeoBridge(Node):
    def __init__(self) -> None:
        super().__init__('racecarneo_bridge')
        self.declare_parameter('frame_prefix', 'racecarneo')
        self.declare_parameter('max_speed', 0.35)
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('publish_lidar', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('camera_fov_degrees', 60.0)
        self.declare_parameter('auto_start_user_program', True)
        self.declare_parameter('auto_start_duration_sec', 20.0)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.42)
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
        self.declare_parameter('publish_actuator_feedback', True)

        self.frame_prefix = self.get_parameter('frame_prefix').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.publish_camera = bool(self.get_parameter('publish_camera').value)
        self.publish_depth = bool(self.get_parameter('publish_depth').value)
        self.publish_lidar = bool(self.get_parameter('publish_lidar').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.camera_fov_degrees = float(self.get_parameter('camera_fov_degrees').value)
        self.auto_start_user_program = bool(self.get_parameter('auto_start_user_program').value)
        self.auto_start_duration_sec = float(self.get_parameter('auto_start_duration_sec').value)
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
        self.publish_actuator_feedback = bool(self.get_parameter('publish_actuator_feedback').value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_update_time = None
        self._last_speed_mps = 0.0
        self._last_yaw_rate = 0.0

        self._cmd_lock = threading.Lock()
        self._speed = 0.0
        self._steering = 0.0
        self._actual_speed_cmd = 0.0
        self._actual_steering_cmd = 0.0
        self._rc = None
        self._stop = False
        self._sim_thread: Optional[threading.Thread] = None
        self._auto_start_thread: Optional[threading.Thread] = None

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
        speed = max(-1.0, min(1.0, float(msg.drive.speed)))
        steering = max(-1.0, min(1.0, float(msg.drive.steering_angle)))
        with self._cmd_lock:
            self._speed = speed
            self._steering = steering

    def start(self) -> None:
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
        sys.argv = [sys.argv[0], '-s', '-h']
        try:
            import racecar_core
            self._rc = racecar_core.create_racecar(isSimulation=True)
            self._rc.set_start_update(self._on_start, self._on_update, self._on_update_slow)
            if self.auto_start_user_program:
                self._auto_start_thread = threading.Thread(target=self._pulse_user_program_start, daemon=True)
                self._auto_start_thread.start()
            self._rc.go()
        except Exception as exc:
            self.get_logger().exception(f'RACECAR Neo simulator bridge stopped: {exc}')
            rclpy.shutdown()

    def _pulse_user_program_start(self) -> None:
        # MIT RacecarSim exposes header 8 as racecar_go. Some simulator builds use
        # this to enter user-program mode; builds that require UI input ignore it.
        sim_ip = os.environ.get('RACECAR_SIM_IP', '127.0.0.1')
        packet = struct.pack('B', 8)
        deadline = time.monotonic() + max(0.0, self.auto_start_duration_sec)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            while not self._stop and time.monotonic() < deadline:
                try:
                    sock.sendto(packet, (sim_ip, 5064))
                except OSError as exc:
                    self.get_logger().debug(f'Unable to send RacecarSim autostart packet: {exc}')
                time.sleep(0.25)

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
        self._rc.drive.set_speed_angle(speed_cmd, steering_cmd)

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
            target_speed = self._speed
            target_steering = self._steering
        if self.manual_override or self.estop_engaged:
            target_speed = 0.0
            if self.estop_engaged:
                target_steering = 0.0

        throttle_alpha = min(1.0, dt / self.throttle_time_constant_s)
        steering_alpha = min(1.0, dt / self.steering_time_constant_s)
        self._actual_speed_cmd += (target_speed - self._actual_speed_cmd) * throttle_alpha
        self._actual_steering_cmd += (target_steering - self._actual_steering_cmd) * steering_alpha

        speed = float(self._actual_speed_cmd) * self.max_speed
        speed *= 1.0 + float(self.rng.normal(0.0, self.wheel_slip_std))
        steering_angle = float(self._actual_steering_cmd) * self.max_steering_angle
        yaw_rate = speed * math.tan(steering_angle) / self.wheelbase if abs(self.wheelbase) > 1e-6 else 0.0
        self._yaw = math.atan2(math.sin(self._yaw + yaw_rate * dt), math.cos(self._yaw + yaw_rate * dt))
        self._x += speed * math.cos(self._yaw) * dt
        self._y += speed * math.sin(self._yaw) * dt
        if self.encoder_quantization_m > 0.0:
            self._x = round(self._x / self.encoder_quantization_m) * self.encoder_quantization_m
            self._y = round(self._y / self.encoder_quantization_m) * self.encoder_quantization_m
        self._last_speed_mps = speed
        self._last_yaw_rate = yaw_rate
        return self._actual_speed_cmd, self._actual_steering_cmd

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
        qx, qy, qz, qw = self._yaw_quaternion()
        transforms = []

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
        transforms.append(odom_to_base)

        sensor_frames = [
            ('camera_color_optical_frame', 0.18, 0.0, 0.22, -math.pi / 2.0, 0.0, -math.pi / 2.0),
            ('camera_depth_optical_frame', 0.18, 0.0, 0.22, -math.pi / 2.0, 0.0, -math.pi / 2.0),
            ('lidar_link', 0.08, 0.0, 0.18, 0.0, 0.0, 0.0),
            ('imu_link', 0.0, 0.0, 0.08, 0.0, 0.0, 0.0),
        ]
        for child, x, y, z, roll, pitch, yaw in sensor_frames:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.base_frame
            tf.child_frame_id = self._frame(child)
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
            cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
            cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
            tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
            tf.transform.rotation.z = cr * cp * sy - sr * sp * cy
            tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
            transforms.append(tf)

        self.tf_broadcaster.sendTransform(transforms)

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
            depth_m += self.rng.normal(0.0, self.depth_noise_std_m, size=depth_m.shape).astype(np.float32)
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
        fx = (width / 2.0) / math.tan(math.radians(self.camera_fov_degrees) / 2.0)
        fy = fx
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
            raw_m += self.rng.normal(0.0, self.lidar_noise_std_m, size=raw_m.shape).astype(np.float32)
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
        accel = np.asarray(self._rc.physics.get_linear_acceleration(), dtype=np.float32)
        gyro = np.asarray(self._rc.physics.get_angular_velocity(), dtype=np.float32)
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
            msg.drive.speed = float(speed_cmd)
            msg.drive.steering_angle = float(steering_cmd)
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
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

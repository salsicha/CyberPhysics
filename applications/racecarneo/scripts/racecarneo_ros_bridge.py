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
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Header


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

        self.frame_prefix = self.get_parameter('frame_prefix').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.publish_camera = bool(self.get_parameter('publish_camera').value)
        self.publish_depth = bool(self.get_parameter('publish_depth').value)
        self.publish_lidar = bool(self.get_parameter('publish_lidar').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.camera_fov_degrees = float(self.get_parameter('camera_fov_degrees').value)
        self.auto_start_user_program = bool(self.get_parameter('auto_start_user_program').value)
        self.auto_start_duration_sec = float(self.get_parameter('auto_start_duration_sec').value)

        self._cmd_lock = threading.Lock()
        self._speed = 0.0
        self._steering = 0.0
        self._rc = None
        self._stop = False
        self._sim_thread: Optional[threading.Thread] = None
        self._auto_start_thread: Optional[threading.Thread] = None

        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 5)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_rect_raw', 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 5)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', 5)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 5)
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

        with self._cmd_lock:
            speed = self._speed
            steering = self._steering
        self._rc.drive.set_speed_angle(speed, steering)

        stamp = self.get_clock().now().to_msg()
        if self.publish_camera:
            self._publish_color(stamp)
            self._publish_camera_info(stamp)
        if self.publish_depth:
            self._publish_depth(stamp)
        if self.publish_lidar:
            self._publish_scan(stamp)
        if self.publish_imu:
            self._publish_imu(stamp)

    def _header(self, stamp, frame: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = f'{self.frame_prefix}/{frame}'
        return header

    def _publish_color(self, stamp) -> None:
        image = self._rc.camera.get_color_image_no_copy()
        if image is None:
            return
        msg = Image()
        msg.header = self._header(stamp, 'camera_color_optical_frame')
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * image.shape[2])
        msg.data = np.ascontiguousarray(image).tobytes()
        self.color_pub.publish(msg)

    def _publish_depth(self, stamp) -> None:
        depth_cm = self._rc.camera.get_depth_image()
        if depth_cm is None:
            return
        depth_m = np.ascontiguousarray(depth_cm.astype(np.float32) / 100.0)
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
        accel = self._rc.physics.get_linear_acceleration()
        gyro = self._rc.physics.get_angular_velocity()
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)


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

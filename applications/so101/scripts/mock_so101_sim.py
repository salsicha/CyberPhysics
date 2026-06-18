#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray

from so101_common import JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS


class MockSO101Sim(Node):
    def __init__(self):
        super().__init__("so101_mock_sim")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.velocities = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.last_time = self.get_clock().now()
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.image_pub = self.create_publisher(Image, "/so101/camera/image_raw", 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/so101/camera/camera_info", 5)
        self.create_subscription(Float64MultiArray, "/so101/joint_commands", self._command, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _command(self, msg):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warning("Expected six SO-101 joint commands")
            return
        self.target = np.clip(np.asarray(msg.data, dtype=np.float32), LOWER_LIMITS, UPPER_LIMITS)

    def _tick(self):
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        error = self.target - self.positions
        step = np.clip(error, -1.2 * dt, 1.2 * dt)
        next_positions = np.clip(self.positions + step, LOWER_LIMITS, UPPER_LIMITS)
        self.velocities = (next_positions - self.positions) / dt
        self.positions = next_positions
        stamp = now.to_msg()
        self._publish_joint_state(stamp)
        self._publish_camera(stamp)

    def _publish_joint_state(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = self.positions.astype(float).tolist()
        msg.velocity = self.velocities.astype(float).tolist()
        self.joint_pub.publish(msg)

    def _publish_camera(self, stamp):
        h = self.camera_height
        w = self.camera_width
        y = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
        x = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
        image = np.zeros((h, w, 3), dtype=np.uint8)
        image[:, :, 0] = (x + int((self.positions[0] + 2.0) * 35.0)) % 255
        image[:, :, 1] = (y + int((self.positions[1] + 2.0) * 35.0)) % 255
        image[:, :, 2] = 90
        cx = int(w * (0.5 + 0.22 * np.sin(float(self.positions[2]))))
        cy = int(h * (0.5 - 0.22 * np.sin(float(self.positions[3]))))
        image[max(0, cy - 12):min(h, cy + 12), max(0, cx - 12):min(w, cx + 12), :] = [255, 220, 30]

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "groot_camera_rgb_optical_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = image.tobytes()
        self.image_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.width = w
        info.height = h
        fx = fy = 554.0
        cx0 = w / 2.0
        cy0 = h / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx0, 0.0, fy, cy0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx0, 0.0, 0.0, fy, cy0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)


def main():
    rclpy.init()
    node = MockSO101Sim()
    try:
        rclpy.spin(node)
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

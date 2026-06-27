#!/usr/bin/env python3

from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import StaticTransformBroadcaster

from so101_common import (
    BASE_FRAME_ID,
    BASE_TO_CAMERA_LINK_RPY,
    BASE_TO_CAMERA_LINK_TRANSLATION,
    CAMERA_FRAME_ID,
    CAMERA_LINK_FRAME_ID,
    CAMERA_LINK_TO_OPTICAL_RPY,
    CAMERA_LINK_TO_OPTICAL_TRANSLATION,
    CAMERA_HEIGHT,
    CAMERA_IMU_TOPIC,
    CAMERA_TO_DEPTH_RPY,
    CAMERA_TO_DEPTH_TRANSLATION,
    CAMERA_WIDTH,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_FRAME_ID,
    DEPTH_TOPIC,
    JOINT_BACKLASH,
    JOINT_NAMES,
    JOINT_QUANTIZATION,
    JOINT_VELOCITY_LIMITS,
    LOWER_LIMITS,
    RGB_CAMERA_INFO_TOPIC,
    RGB_TOPIC,
    TABLE_FRAME_ID,
    UPPER_LIMITS,
    WORLD_TO_TABLE_RPY,
    WORLD_TO_TABLE_TRANSLATION,
    camera_info_values,
    quantize,
    quaternion_from_euler,
    synthetic_rgbd,
)


class MockSO101Sim(Node):
    def __init__(self):
        super().__init__("so101_mock_sim")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("camera_width", CAMERA_WIDTH)
        self.declare_parameter("camera_height", CAMERA_HEIGHT)
        self.declare_parameter("command_latency_s", 0.04)
        self.declare_parameter("velocity_scale", 1.0)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("joint_noise_std", 0.0004)
        self.positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.velocities = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.command_queue = deque()
        self.last_time = self.get_clock().now()
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.command_latency_s = float(self.get_parameter("command_latency_s").value)
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.joint_noise_std = float(self.get_parameter("joint_noise_std").value)
        self.rng = np.random.default_rng(101)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.image_pub = self.create_publisher(Image, RGB_TOPIC, 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, RGB_CAMERA_INFO_TOPIC, 5)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, DEPTH_CAMERA_INFO_TOPIC, 5)
        self.imu_pub = self.create_publisher(Imu, CAMERA_IMU_TOPIC, 20)
        self.create_subscription(Float64MultiArray, "/so101/joint_commands", self._command, 10)

        if bool(self.get_parameter("publish_static_tf").value):
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform(self._static_transforms())

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self._tick
        )

    def _command(self, msg):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warning("Expected six SO-101 joint commands")
            return
        target = np.clip(np.asarray(msg.data, dtype=np.float32), LOWER_LIMITS, UPPER_LIMITS)
        due_time = self.get_clock().now().nanoseconds * 1e-9 + self.command_latency_s
        self.command_queue.append((due_time, target))

    def _tick(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        while self.command_queue and self.command_queue[0][0] <= now_s:
            _, self.target = self.command_queue.popleft()

        error = self.target - self.positions
        active_error = np.where(np.abs(error) < JOINT_BACKLASH, 0.0, error)
        max_step = JOINT_VELOCITY_LIMITS * self.velocity_scale * dt
        step = np.clip(active_error, -max_step, max_step)
        next_positions = np.clip(self.positions + step, LOWER_LIMITS, UPPER_LIMITS)
        self.velocities = (next_positions - self.positions) / dt
        self.positions = next_positions

        stamp = now.to_msg()
        measured = self._measured_positions()
        self._publish_joint_state(stamp, measured)
        self._publish_camera(stamp, measured)
        self._publish_imu(stamp)

    def _measured_positions(self):
        noise = self.rng.normal(0.0, self.joint_noise_std, size=len(JOINT_NAMES)).astype(np.float32)
        noise[-1] *= 0.2
        return np.clip(quantize(self.positions + noise, JOINT_QUANTIZATION), LOWER_LIMITS, UPPER_LIMITS)

    def _publish_joint_state(self, stamp, measured_positions):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = measured_positions.astype(float).tolist()
        msg.velocity = quantize(self.velocities, JOINT_QUANTIZATION).astype(float).tolist()
        effort_fraction = np.minimum(
            1.0,
            np.abs(self.target - self.positions) / np.maximum(UPPER_LIMITS - LOWER_LIMITS, 1e-6),
        )
        msg.effort = effort_fraction.astype(float).tolist()
        self.joint_pub.publish(msg)

    def _publish_camera(self, stamp, measured_positions):
        image, depth = synthetic_rgbd(measured_positions, self.camera_width, self.camera_height)
        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = CAMERA_FRAME_ID
        image_msg.height = self.camera_height
        image_msg.width = self.camera_width
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = 0
        image_msg.step = self.camera_width * 3
        image_msg.data = image.tobytes()
        self.image_pub.publish(image_msg)

        self.camera_info_pub.publish(self._camera_info(stamp, CAMERA_FRAME_ID))

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = DEPTH_FRAME_ID
        depth_msg.height = self.camera_height
        depth_msg.width = self.camera_width
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = self.camera_width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self._camera_info(stamp, DEPTH_FRAME_ID))

    def _publish_imu(self, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME_ID
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.z = float(self.velocities[0])
        msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0008]
        msg.linear_acceleration.z = 9.80665
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
        self.imu_pub.publish(msg)

    def _camera_info(self, stamp, frame_id):
        values = camera_info_values(self.camera_width, self.camera_height)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = values["width"]
        info.height = values["height"]
        info.distortion_model = values["distortion_model"]
        info.d = values["d"]
        info.k = values["k"]
        info.r = values["r"]
        info.p = values["p"]
        return info

    def _static_transforms(self):
        return [
            self._transform("world", TABLE_FRAME_ID, WORLD_TO_TABLE_TRANSLATION, WORLD_TO_TABLE_RPY),
            self._transform(BASE_FRAME_ID, CAMERA_LINK_FRAME_ID, BASE_TO_CAMERA_LINK_TRANSLATION, BASE_TO_CAMERA_LINK_RPY),
            self._transform(CAMERA_LINK_FRAME_ID, CAMERA_FRAME_ID, CAMERA_LINK_TO_OPTICAL_TRANSLATION, CAMERA_LINK_TO_OPTICAL_RPY),
            self._transform(CAMERA_FRAME_ID, DEPTH_FRAME_ID, CAMERA_TO_DEPTH_TRANSLATION, CAMERA_TO_DEPTH_RPY),
        ]

    def _transform(self, parent, child, translation, rpy):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        qx, qy, qz, qw = quaternion_from_euler(*rpy)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg


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

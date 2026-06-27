#!/usr/bin/env python3
"""Validate RACECAR Neo simulation stack topic freshness and TF connectivity."""

import argparse
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


TOPIC_TYPES = {
    '/camera/color/image_raw': Image,
    '/camera/color/camera_info': CameraInfo,
    '/camera/depth/image_rect_raw': Image,
    '/depth_anything/depth/image': Image,
    '/scan': LaserScan,
    '/imu/data_raw': Imu,
    '/odom': Odometry,
    '/ackermann_feedback': AckermannDriveStamped,
}
NVBLOX_TOPICS = [
    '/nvblox_node/static_map_slice',
    '/nvblox_node/combined_map_slice',
]
REQUIRED_TF = [
    ('map', 'odom'),
    ('odom', 'base_link'),
    ('base_link', 'racecarneo/camera_color_optical_frame'),
    ('base_link', 'racecarneo/camera_depth_optical_frame'),
    ('base_link', 'racecarneo/lidar_link'),
    ('base_link', 'racecarneo/imu_link'),
]


class StackValidator(Node):
    def __init__(self, freshness_s):
        super().__init__('racecarneo_stack_validator')
        self.freshness_s = freshness_s
        self.last_seen = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for topic, msg_type in TOPIC_TYPES.items():
            self.create_subscription(msg_type, topic, self._seen(topic), 10)

    def _seen(self, topic):
        def callback(_msg):
            self.last_seen[topic] = time.monotonic()
        return callback

    def missing_topics(self):
        now = time.monotonic()
        missing = []
        for topic in TOPIC_TYPES:
            seen = self.last_seen.get(topic)
            if seen is None or now - seen > self.freshness_s:
                missing.append(topic)
        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        missing.extend(topic for topic in NVBLOX_TOPICS if topic not in topic_names)
        return missing

    def missing_transforms(self):
        missing = []
        for target, source in REQUIRED_TF:
            try:
                self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.2))
            except TransformException:
                missing.append(f'{target} <- {source}')
        return missing


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-s', type=float, default=60.0)
    parser.add_argument('--freshness-s', type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = StackValidator(args.freshness_s)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing_topics = node.missing_topics()
            missing_tf = node.missing_transforms()
            if not missing_topics and not missing_tf:
                node.get_logger().info('RACECAR Neo stack validation passed')
                return
        raise SystemExit(
            'RACECAR Neo stack validation failed; '
            f'missing/stale topics={node.missing_topics()} missing_tf={node.missing_transforms()}'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

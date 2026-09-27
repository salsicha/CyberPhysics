#!/usr/bin/env python3
"""Validate RACECAR Neo simulation stack topics, TF connectivity, and Nav2 state."""

import argparse
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_srvs.srv import Trigger
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
# This image has no nvblox_msgs, so these are checked for a live publisher
# rather than subscribed to. A topic name alone proves nothing: Nav2's costmap
# subscription creates it even when nvblox is down.
NVBLOX_TOPICS = [
    '/nvblox_node/static_map_slice',
]
NAV2_LIFECYCLE_MANAGER = '/lifecycle_manager_navigation'
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
        self.nav2_active = False
        self._nav2_future = None
        self._nav2_client = self.create_client(Trigger, f'{NAV2_LIFECYCLE_MANAGER}/is_active')

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
        missing.extend(topic for topic in NVBLOX_TOPICS if self.count_publishers(topic) == 0)
        return missing

    def poll_nav2(self):
        """Ask Nav2's lifecycle manager whether every managed node is active."""
        if self._nav2_future is not None:
            if not self._nav2_future.done():
                return
            result = self._nav2_future.result()
            self.nav2_active = bool(result is not None and result.success)
            self._nav2_future = None
        if self._nav2_client.service_is_ready():
            self._nav2_future = self._nav2_client.call_async(Trigger.Request())
        else:
            self.nav2_active = False

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
            node.poll_nav2()
            missing_topics = node.missing_topics()
            missing_tf = node.missing_transforms()
            if not missing_topics and not missing_tf and node.nav2_active:
                node.get_logger().info('RACECAR Neo stack validation passed')
                return
        raise SystemExit(
            'RACECAR Neo stack validation failed; '
            f'missing/stale topics={node.missing_topics()} missing_tf={node.missing_transforms()} '
            f'nav2_active={node.nav2_active}'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

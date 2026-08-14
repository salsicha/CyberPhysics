#!/usr/bin/env python3
"""Validate Aerodrone GPS, DemNav, WildNav, and fused navigation topics."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    FluidPressure,
    Image,
    Imu,
    MagneticField,
    NavSatFix,
    PointCloud2,
    Range,
)
from std_msgs.msg import Bool, Float32, Int32, String


TOPIC_TYPES = {
    '/navigation/gps_standard': NavSatFix,
    '/aerodrone/imu/data_raw': Imu,
    '/aerodrone/compass/magnetic_field': MagneticField,
    '/aerodrone/barometer': FluidPressure,
    '/oak1/image_highres': Image,
    '/oak1/image_highres/camera_info': CameraInfo,
    '/oak1/relative_depth': Image,
    '/oak1/camera_info': CameraInfo,
    '/aerodrone/rangefinder': Range,
    '/aerodrone/battery': BatteryState,
    '/aerodrone/rc_failsafe': Bool,
    '/aerodrone/geofence_violation': Bool,
    '/aerodrone/emergency_land': Bool,
    '/demnav/fix': NavSatFix,
    '/demnav/odometry': Odometry,
    '/demnav/metric_depth': Image,
    '/demnav/points': PointCloud2,
    '/demnav/confidence': Float32,
    '/demnav/depth_scale': Float32,
    '/demnav/valid': Bool,
    '/wildnav/fix': NavSatFix,
    '/wildnav/odometry': Odometry,
    '/wildnav/confidence': Float32,
    '/wildnav/match_count': Int32,
    '/wildnav/valid': Bool,
    '/navigation/odometry': Odometry,
    '/navigation/fix': NavSatFix,
    '/navigation/correction_source': String,
    '/navigation/correction_confidence': Float32,
}


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


class AerodroneNavsimValidator(Node):
    def __init__(self, freshness_s):
        super().__init__('aerodrone_navsim_validator')
        self.freshness_s = freshness_s
        self.last_seen = {}
        self.invalid = {}
        for topic, msg_type in TOPIC_TYPES.items():
            self.create_subscription(msg_type, topic, self._seen(topic), 10)

    def _seen(self, topic):
        def callback(msg):
            reason = self._validate_message(msg)
            if reason:
                self.invalid[topic] = reason
            else:
                self.invalid.pop(topic, None)
                self.last_seen[topic] = time.monotonic()

        return callback

    def _validate_message(self, msg):
        if isinstance(msg, NavSatFix):
            if not _finite([msg.latitude, msg.longitude, msg.altitude]):
                return 'non-finite GPS fix'
            if not _finite(msg.position_covariance):
                return 'non-finite GPS covariance'
        elif isinstance(msg, Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if not _finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
                return 'non-finite odometry pose'
            if not _finite(msg.pose.covariance):
                return 'non-finite odometry covariance'
        elif isinstance(msg, Image):
            if msg.width <= 0 or msg.height <= 0:
                return 'empty image dimensions'
            if not msg.data:
                return 'empty image data'
        elif isinstance(msg, CameraInfo):
            if msg.width <= 0 or msg.height <= 0:
                return 'empty camera_info dimensions'
            if not _finite(msg.k):
                return 'non-finite camera_info intrinsics'
        elif isinstance(msg, Range):
            if not _finite([msg.range, msg.min_range, msg.max_range]):
                return 'non-finite rangefinder value'
            if msg.min_range > msg.max_range:
                return 'invalid rangefinder limits'
        elif isinstance(msg, BatteryState):
            if not _finite([msg.voltage, msg.current, msg.percentage]):
                return 'non-finite battery state'
        elif isinstance(msg, Imu):
            av = msg.angular_velocity
            la = msg.linear_acceleration
            if not _finite([av.x, av.y, av.z, la.x, la.y, la.z]):
                return 'non-finite IMU values'
        elif isinstance(msg, MagneticField):
            f = msg.magnetic_field
            if not _finite([f.x, f.y, f.z]):
                return 'non-finite magnetic field'
        elif isinstance(msg, FluidPressure):
            if not _finite([msg.fluid_pressure, msg.variance]):
                return 'non-finite barometer pressure'
            if msg.fluid_pressure <= 0.0:
                return 'non-positive barometer pressure'
        elif isinstance(msg, Float32):
            if not math.isfinite(float(msg.data)):
                return 'non-finite scalar'
        elif isinstance(msg, Int32):
            if msg.data < 0:
                return 'negative count'
        elif isinstance(msg, PointCloud2):
            if msg.width == 0 and msg.height == 0:
                return 'empty point cloud dimensions'
        return None

    def missing_or_stale(self):
        now = time.monotonic()
        missing = []
        for topic in TOPIC_TYPES:
            seen = self.last_seen.get(topic)
            if seen is None or now - seen > self.freshness_s:
                missing.append(topic)
        return missing


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-s', type=float, default=180.0)
    parser.add_argument('--freshness-s', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = AerodroneNavsimValidator(args.freshness_s)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.invalid and not node.missing_or_stale():
                node.get_logger().info('Aerodrone navsim validation passed')
                return
        raise SystemExit(
            'Aerodrone navsim validation failed; '
            f'missing/stale topics={node.missing_or_stale()} '
            f'invalid={node.invalid}'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

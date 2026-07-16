#!/usr/bin/env python3
import copy
import math
from collections import deque
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String
from synthetic_world import local_to_latlon


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class Correction:
    valid: bool = False
    confidence: float = 0.0
    east: float = 0.0
    north: float = 0.0
    variance: float = 10000.0
    received_ns: int = 0
    pending: object = None
    processed_stamp_ns: int = -1
    fix: object = None


class NavigationFusionNode(Node):
    def __init__(self):
        super().__init__('navigation_fusion_node')
        defaults = {
            'raw_odom_topic': '/drone0/sensor_measurements/odom',
            'demnav_odom_topic': '/demnav/odometry',
            'demnav_fix_topic': '/demnav/fix',
            'demnav_confidence_topic': '/demnav/confidence',
            'demnav_valid_topic': '/demnav/valid',
            'wildnav_odom_topic': '/wildnav/odometry',
            'wildnav_fix_topic': '/wildnav/fix',
            'wildnav_confidence_topic': '/wildnav/confidence',
            'wildnav_valid_topic': '/wildnav/valid',
            'output_topic': '/navigation/odometry',
            'fix_output_topic': '/navigation/fix',
            'source_topic': '/navigation/correction_source',
            'confidence_topic': '/navigation/correction_confidence',
            'minimum_demnav_confidence': 0.35,
            'minimum_wildnav_confidence': 0.20,
            'correction_timeout_s': 30.0,
            'correction_time_constant_s': 2.0,
            'maximum_correction_m': 300.0,
            'raw_history_s': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.raw_history = deque()
        self.latest_raw = None
        self.demnav = Correction()
        self.wildnav = Correction()
        self.offset_east = 0.0
        self.offset_north = 0.0
        self.last_publish_ns = None
        self.global_reference = None

        self.output_pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter('fix_output_topic').value, 10)
        self.source_pub = self.create_publisher(
            String, self.get_parameter('source_topic').value, 10)
        self.confidence_pub = self.create_publisher(
            Float32, self.get_parameter('confidence_topic').value, 10)

        self.create_subscription(
            Odometry, self.get_parameter('raw_odom_topic').value,
            self._on_raw, qos_profile_sensor_data)
        self._create_source_subscriptions('demnav', self.demnav)
        self._create_source_subscriptions('wildnav', self.wildnav)
        self.get_logger().info(
            'Navigation fusion ready; publishing corrected high-rate odometry')

    def _create_source_subscriptions(self, name, state):
        self.create_subscription(
            Bool, self.get_parameter(f'{name}_valid_topic').value,
            lambda msg, source=name, target=state:
            self._on_valid(source, target, msg), 10)
        self.create_subscription(
            Float32, self.get_parameter(f'{name}_confidence_topic').value,
            lambda msg, source=name, target=state:
            self._on_confidence(source, target, msg), 10)
        self.create_subscription(
            Odometry, self.get_parameter(f'{name}_odom_topic').value,
            lambda msg, source=name, target=state:
            self._on_correction(source, target, msg), 10)
        self.create_subscription(
            NavSatFix, self.get_parameter(f'{name}_fix_topic').value,
            lambda msg, source=name, target=state:
            self._on_fix(source, target, msg), 10)

    def _on_valid(self, source, state, msg):
        state.valid = msg.data
        if state.valid:
            self._accept_pending(source, state)

    def _on_confidence(self, source, state, msg):
        state.confidence = float(msg.data)
        self._accept_pending(source, state)

    def _on_raw(self, msg):
        self.latest_raw = msg
        message_time = stamp_ns(msg.header.stamp)
        if message_time == 0:
            message_time = self.get_clock().now().nanoseconds
        self.raw_history.append((
            message_time,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y))
        history_ns = int(
            float(self.get_parameter('raw_history_s').value) * 1e9)
        while (
                self.raw_history and
                message_time - self.raw_history[0][0] > history_ns):
            self.raw_history.popleft()

        target_east, target_north, source, confidence, variance = (
            self._target_offset())
        if source:
            if self.last_publish_ns is None:
                alpha = 1.0
            else:
                dt = max(0.0, (message_time - self.last_publish_ns) / 1e9)
                tau = max(
                    0.01,
                    float(self.get_parameter(
                        'correction_time_constant_s').value))
                alpha = 1.0 - math.exp(-dt / tau)
            self.offset_east += alpha * (target_east - self.offset_east)
            self.offset_north += alpha * (target_north - self.offset_north)
        self.last_publish_ns = message_time

        fused = copy.deepcopy(msg)
        fused.header.frame_id = 'odom'
        fused.pose.pose.position.x += self.offset_east
        fused.pose.pose.position.y += self.offset_north
        if source:
            fused.pose.covariance[0] = max(
                fused.pose.covariance[0], variance)
            fused.pose.covariance[7] = max(
                fused.pose.covariance[7], variance)
        self.output_pub.publish(fused)
        self._publish_fix(fused)
        self.source_pub.publish(String(data=source or 'dead_reckoning'))
        self.confidence_pub.publish(Float32(data=confidence))

    def _on_correction(self, source, state, msg):
        state.pending = msg
        self._accept_pending(source, state)

    def _on_fix(self, source, state, msg):
        state.fix = msg
        self._update_global_reference(source, state)

    def _accept_pending(self, source, state):
        msg = state.pending
        if self.latest_raw is None or not state.valid:
            return
        if msg is None:
            return
        message_stamp = stamp_ns(msg.header.stamp)
        if message_stamp == state.processed_stamp_ns:
            return
        minimum = float(
            self.get_parameter(f'minimum_{source}_confidence').value)
        if state.confidence < minimum:
            return
        correction_time = stamp_ns(msg.header.stamp)
        raw_east, raw_north = self._raw_at(correction_time)
        east = msg.pose.pose.position.x - raw_east
        north = msg.pose.pose.position.y - raw_north
        magnitude = math.hypot(east, north)
        if magnitude > float(
                self.get_parameter('maximum_correction_m').value):
            self.get_logger().warning(
                f'Rejected {source} correction of {magnitude:.1f} m')
            return
        covariance = msg.pose.covariance
        state.east = east
        state.north = north
        state.variance = max(1.0, 0.5 * (covariance[0] + covariance[7]))
        state.received_ns = self.get_clock().now().nanoseconds
        state.processed_stamp_ns = message_stamp
        self._update_global_reference(source, state)

    def _update_global_reference(self, source, state):
        if state.fix is None or state.pending is None:
            return
        fix_stamp = stamp_ns(state.fix.header.stamp)
        odom_stamp = stamp_ns(state.pending.header.stamp)
        if state.processed_stamp_ns != odom_stamp:
            return
        if fix_stamp and odom_stamp and abs(fix_stamp - odom_stamp) > 1_000_000_000:
            return
        position = state.pending.pose.pose.position
        self.global_reference = (
            state.fix.latitude,
            state.fix.longitude,
            state.fix.altitude,
            position.x,
            position.y,
            position.z,
            source,
        )

    def _publish_fix(self, fused):
        if self.global_reference is None:
            return
        lat, lon, alt, ref_east, ref_north, ref_up, _ = self.global_reference
        east = fused.pose.pose.position.x - ref_east
        north = fused.pose.pose.position.y - ref_north
        fix = NavSatFix()
        fix.header = fused.header
        fix.header.frame_id = 'map'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = local_to_latlon(east, north, lat, lon)
        fix.altitude = alt + fused.pose.pose.position.z - ref_up
        fix.position_covariance = [
            fused.pose.covariance[0], 0.0, 0.0,
            0.0, fused.pose.covariance[7], 0.0,
            0.0, 0.0, fused.pose.covariance[14],
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

    def _raw_at(self, target_ns):
        if not self.raw_history or target_ns == 0:
            return (
                self.latest_raw.pose.pose.position.x,
                self.latest_raw.pose.pose.position.y)
        _, east, north = min(
            self.raw_history, key=lambda item: abs(item[0] - target_ns))
        return east, north

    def _target_offset(self):
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(
            float(self.get_parameter('correction_timeout_s').value) * 1e9)
        active = []
        for name, state in (
                ('demnav', self.demnav), ('wildnav', self.wildnav)):
            minimum = float(
                self.get_parameter(f'minimum_{name}_confidence').value)
            if (
                    state.valid and state.confidence >= minimum and
                    state.received_ns > 0 and
                    now - state.received_ns <= timeout_ns):
                weight = state.confidence * state.confidence / state.variance
                active.append((name, state, weight))
        if not active:
            return self.offset_east, self.offset_north, '', 0.0, 0.0
        total = sum(item[2] for item in active)
        east = sum(item[1].east * item[2] for item in active) / total
        north = sum(item[1].north * item[2] for item in active) / total
        source = '+'.join(item[0] for item in active)
        confidence = sum(
            item[1].confidence * item[2] for item in active) / total
        variance = 1.0 / total
        return east, north, source, confidence, variance


def main(args=None):
    rclpy.init(args=args)
    node = NavigationFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

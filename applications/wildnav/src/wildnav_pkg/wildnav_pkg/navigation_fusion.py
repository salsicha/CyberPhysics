#!/usr/bin/env python3
"""Fuse sparse absolute DemNav/WildNav fixes with high-rate raw odometry.

Frames and time
---------------
Raw odometry (MAVROS ``local_position/odom``) is local ENU about the
autopilot's EKF origin. That origin is the single shared anchor of the
navigation stack: it arrives on ``origin_topic`` (``/navigation/origin``,
geographic_msgs/GeoPointStamped, reliable + transient_local, published by
``applications/aerostack2/scripts/ardupilot_navigation_bridge.py`` from MAVROS
``global_position/gp_origin``; altitude is WGS-84 ellipsoid height).

DemNav and WildNav publish absolute ``NavSatFix`` fixes stamped with the
camera frame time. Each accepted fix is projected into that local ENU frame
with ``synthetic_world.latlon_to_local`` (the one projection every node uses)
and compared with the raw odometry *at the fix stamp*. A fix whose stamp has
no raw sample within ``max_time_offset_s`` is rejected: comparing it with
whatever sample happens to be nearest silently mixes positions from
different times (or clock domains, e.g. simulator time against wall time).
Without an origin no correction is possible and the node publishes raw
odometry flagged ``dead_reckoning``.

Simulators that publish ``/clock`` (Gazebo, Isaac) stamp camera frames in
simulator time while MAVROS stamps odometry in wall time. The node records
(simulator time, wall time) pairs from ``sim_clock_topic`` and maps a fix
stamp that lies in the simulator clock's recent history onto wall time by
interpolation before matching. Without ``/clock`` (hardware) nothing is
translated.

The dead-reckoning *age* below is measured on that simulator clock while it
runs (a simulator slower than real time must not age the estimate faster
than the vehicle flies), and in wall time without it.

Outputs
-------
``output_topic``: raw odometry plus the smoothed horizontal correction
offset (no offset while the autopilot itself absorbs the corrections, see
below). Horizontal position variance is the larger of the raw variance and
``correction variance + (dead_reckoning_drift_mps * age)^2``, where *age* is
the time since the last accepted correction (since the origin arrived when
there has been none), so consumers such as the DemNav/WildNav search window
see the uncertainty grow while no correction arrives.

``<prefix>/fix`` (``fix_output_topic``): the same estimate as a NavSatFix in
the origin's altitude datum with a known diagonal covariance (m^2), at the
raw odometry rate. Status semantics:

- ``STATUS_FIX`` (0): usable estimate; corrections are recent, or dead
  reckoning has lasted at most ``dead_reckoning_limit_s``.
- ``STATUS_NO_FIX`` (-1): dead reckoning beyond ``dead_reckoning_limit_s``;
  the estimate is still published (with its large covariance) but must not
  be used as a prior.

No fix is published before the origin is known. ``STATUS_GBAS_FIX`` is never
claimed: the estimate is not ground-based-augmentation quality.

``<prefix>/correction_source``: ``demnav``, ``wildnav``, ``demnav+wildnav``
while accepted corrections are active, else ``dead_reckoning``.

``<prefix>/correction_fix``: each accepted absolute fix exactly once,
propagated from its camera time to the latest raw sample with the raw
odometry *velocity* (position jumps the autopilot makes in between do not
leak into it) and stamped with that sample's time. This is what the
autopilot bridge forwards to ArduPilot as GPS_INPUT. It is never the fused
estimate: that is the autopilot's own odometry plus an offset, and feeding it
back would make the EKF chase its own offset forever.

When the bridge reports on ``<prefix>/autopilot_corrections_active`` (Bool,
transient_local) that ArduPilot is receiving the corrections, the EKF itself
absorbs them, so this node stops applying offsets (raw odometry then already
contains the corrections; applying them again would double-correct).
"""
import bisect
import copy
import math
from dataclasses import dataclass

import rclpy
from geographic_msgs.msg import GeoPointStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String
from synthetic_world import latlon_to_local, local_to_latlon

SOURCES = ('demnav', 'wildnav')
DEAD_RECKONING = 'dead_reckoning'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamp_from_ns(template, value_ns):
    """Copy a header stamp and set it to value_ns."""
    stamp = copy.deepcopy(template)
    stamp.sec = int(value_ns // 1_000_000_000)
    stamp.nanosec = int(value_ns % 1_000_000_000)
    return stamp


def finite_odometry(msg):
    """Validate all numeric fields copied into fused odometry."""
    pose, twist = msg.pose.pose, msg.twist.twist
    vectors = (pose.position, twist.linear, twist.angular)
    values = [value for vector in vectors for value in (vector.x, vector.y, vector.z)]
    q = pose.orientation
    values.extend((q.x, q.y, q.z, q.w))
    values.extend(msg.pose.covariance)
    values.extend(msg.twist.covariance)
    return (all(math.isfinite(value) for value in values)
            and all(msg.pose.covariance[index] >= 0 for index in (0, 7, 14)))


def fix_horizontal_variance(fix):
    """Horizontal variance (m^2) of a NavSatFix, or None when unknown/invalid."""
    if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    variance = max(fix.position_covariance[0], fix.position_covariance[4])
    if not math.isfinite(variance) or variance <= 0.0:
        return None
    return float(variance)


def enu_velocity(msg):
    """Horizontal ENU velocity of an odometry message.

    MAVROS reports twist in the child (body) frame; rotate it by the pose
    orientation. Twist already in the parent frame is used as is.
    """
    linear = msg.twist.twist.linear
    child = str(getattr(msg, 'child_frame_id', '') or '')
    if not child or child == str(msg.header.frame_id or ''):
        return float(linear.x), float(linear.y)
    q = msg.pose.pose.orientation
    x, y, z = linear.x, linear.y, linear.z
    # Rotate the body vector by quaternion q (world <- body).
    r00 = 1 - 2 * (q.y * q.y + q.z * q.z)
    r01 = 2 * (q.x * q.y - q.z * q.w)
    r02 = 2 * (q.x * q.z + q.y * q.w)
    r10 = 2 * (q.x * q.y + q.z * q.w)
    r11 = 1 - 2 * (q.x * q.x + q.z * q.z)
    r12 = 2 * (q.y * q.z - q.x * q.w)
    return (r00 * x + r01 * y + r02 * z, r10 * x + r11 * y + r12 * z)


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


class NavigationFusionNode(Node):
    def __init__(self):
        super().__init__('navigation_fusion_node')
        defaults = {
            'raw_odom_topic': '/drone0/sensor_measurements/odom',
            'origin_topic': '/navigation/origin',
            'demnav_fix_topic': '/demnav/fix',
            'demnav_confidence_topic': '/demnav/confidence',
            'demnav_valid_topic': '/demnav/valid',
            'wildnav_fix_topic': '/wildnav/fix',
            'wildnav_confidence_topic': '/wildnav/confidence',
            'wildnav_valid_topic': '/wildnav/valid',
            'output_topic': '/navigation/odometry',
            # Empty: derived from output_topic's namespace, e.g.
            # /airplane/navigation/odometry -> /airplane/navigation/fix.
            'fix_output_topic': '',
            'source_topic': '',
            'confidence_topic': '',
            'correction_output_topic': '',
            'autopilot_corrections_topic': '',
            'minimum_demnav_confidence': 0.35,
            'minimum_wildnav_confidence': 0.20,
            'correction_timeout_s': 30.0,
            'correction_time_constant_s': 2.0,
            'maximum_correction_m': 300.0,
            'raw_history_s': 20.0,
            'max_time_offset_s': 0.25,
            'max_interpolation_gap_s': 1.5,
            'max_fix_age_s': 10.0,
            'uncorrected_variance_m2': 100.0,
            'dead_reckoning_drift_mps': 0.5,
            'dead_reckoning_limit_s': 300.0,
            'propagation_velocity_sigma_mps': 0.3,
            'sim_clock_topic': '/clock',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.raw_history = []
        self.clock_history = []
        self.latest_raw = None
        self.demnav = Correction()
        self.wildnav = Correction()
        self.offset_east = 0.0
        self.offset_north = 0.0
        self.last_publish_ns = None
        self.origin = None
        self.origin_time = None
        self.last_correction_time = None
        self.sim_now_ns = None
        self.last_correction_variance = None
        self.autopilot_absorbs = False

        output_topic = self.get_parameter('output_topic').value
        self.output_pub = self.create_publisher(Odometry, output_topic, 10)
        self.fix_pub = self.create_publisher(
            NavSatFix, self._derived_topic('fix_output_topic', 'fix'), 10)
        self.source_pub = self.create_publisher(
            String, self._derived_topic('source_topic', 'correction_source'), 10)
        self.confidence_pub = self.create_publisher(
            Float32,
            self._derived_topic('confidence_topic', 'correction_confidence'), 10)
        self.correction_pub = self.create_publisher(
            NavSatFix,
            self._derived_topic('correction_output_topic', 'correction_fix'), 10)

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GeoPointStamped, self.get_parameter('origin_topic').value,
            self._on_origin, latched)
        self.create_subscription(
            Bool, self._derived_topic(
                'autopilot_corrections_topic', 'autopilot_corrections_active'),
            self._on_autopilot_corrections, latched)
        self.create_subscription(
            Odometry, self.get_parameter('raw_odom_topic').value,
            self._on_raw, qos_profile_sensor_data)
        clock_topic = str(self.get_parameter('sim_clock_topic').value or '')
        if clock_topic:
            self.create_subscription(
                Clock, clock_topic, self._on_clock, qos_profile_sensor_data)
        self._create_source_subscriptions('demnav', self.demnav)
        self._create_source_subscriptions('wildnav', self.wildnav)
        self.get_logger().info(
            'Navigation fusion ready; waiting for the EKF origin on '
            f"{self.get_parameter('origin_topic').value}")

    def _derived_topic(self, name, leaf):
        value = str(self.get_parameter(name).value or '')
        if value:
            return value
        output = str(self.get_parameter('output_topic').value)
        prefix = output.rsplit('/', 1)[0] if '/' in output else ''
        return f'{prefix}/{leaf}'

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
            NavSatFix, self.get_parameter(f'{name}_fix_topic').value,
            lambda msg, source=name, target=state:
            self._on_fix(source, target, msg), 10)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _on_origin(self, msg):
        origin = (float(msg.position.latitude), float(msg.position.longitude),
                  float(msg.position.altitude))
        if (not all(math.isfinite(value) for value in origin)
                or not -90.0 <= origin[0] <= 90.0
                or not -180.0 <= origin[1] <= 180.0):
            self.get_logger().warning('Rejected invalid navigation origin')
            return
        if origin == self.origin:
            return
        if self.origin is not None:
            # Corrections and offsets were expressed about the old origin.
            self.get_logger().warning(
                'EKF origin changed; discarding corrections made about the '
                'previous origin')
            self._reset_corrections()
        self.origin = origin
        self.origin_time = self._now()
        self.get_logger().info(
            f'Navigation origin {origin[0]:.7f}, {origin[1]:.7f}, '
            f'{origin[2]:.2f} m')

    def _on_clock(self, msg):
        """Record (simulator ns, wall ns) pairs, at most every 10 ms."""
        sim_ns = stamp_ns(msg.clock)
        wall_ns = self.get_clock().now().nanoseconds
        self.sim_now_ns = sim_ns
        if self.clock_history:
            last_sim = self.clock_history[-1][0]
            if sim_ns < last_sim:
                self.clock_history.clear()  # simulator reset
            elif sim_ns - last_sim < 10_000_000:
                return
        self.clock_history.append((sim_ns, wall_ns))
        history_ns = int(float(self.get_parameter('raw_history_s').value) * 1e9)
        cut = 0
        while (cut < len(self.clock_history)
               and sim_ns - self.clock_history[cut][0] > history_ns):
            cut += 1
        if cut:
            del self.clock_history[:cut]

    def _sim_to_wall(self, sim_ns):
        """Wall time of a simulator-clock stamp, or None if it is outside the
        recorded simulator clock history."""
        history = self.clock_history
        if len(history) < 2 or not history[0][0] <= sim_ns <= history[-1][0]:
            return None
        index = bisect.bisect_left([item[0] for item in history], sim_ns)
        if history[index][0] == sim_ns:
            return history[index][1]
        (s0, w0), (s1, w1) = history[index - 1], history[index]
        return int(w0 + (w1 - w0) * (sim_ns - s0) / (s1 - s0))

    def _on_autopilot_corrections(self, msg):
        active = bool(msg.data)
        if active != self.autopilot_absorbs:
            self.get_logger().info(
                'Autopilot is absorbing corrections; publishing its odometry '
                'without offsets' if active else
                'Autopilot is not receiving corrections; applying offsets')
        self.autopilot_absorbs = active

    def _reset_corrections(self):
        self.demnav = Correction(valid=self.demnav.valid,
                                 confidence=self.demnav.confidence)
        self.wildnav = Correction(valid=self.wildnav.valid,
                                  confidence=self.wildnav.confidence)
        self.offset_east = self.offset_north = 0.0
        self.last_publish_ns = None
        self.last_correction_time = None
        self.last_correction_variance = None

    def _now(self):
        """(wall ns, simulator ns or None when no /clock has arrived)."""
        return self.get_clock().now().nanoseconds, self.sim_now_ns

    def _elapsed_s(self, since):
        """Seconds since a _now() reading: on the simulator clock when both
        readings have it and it has not been reset, else in wall time."""
        wall_ns, sim_ns = self._now()
        if sim_ns is not None and since[1] is not None and sim_ns >= since[1]:
            return (sim_ns - since[1]) / 1e9
        return max(0.0, (wall_ns - since[0]) / 1e9)

    def _state(self, source):
        return self.demnav if source == 'demnav' else self.wildnav

    def _on_valid(self, source, state, msg):
        state.valid = bool(msg.data)
        if state.valid:
            self._accept_pending(source, state)

    def _on_confidence(self, source, state, msg):
        confidence = float(msg.data)
        state.confidence = confidence if math.isfinite(confidence) and 0 <= confidence <= 1 else 0.0
        if state.confidence == 0:
            state.received_ns = 0
        self._accept_pending(source, state)

    def _on_fix(self, source, state, msg):
        state.pending = msg
        self._accept_pending(source, state)

    def _on_raw(self, msg):
        if not finite_odometry(msg):
            self.get_logger().warning('Rejected non-finite or invalid raw odometry')
            return
        if not all(math.isfinite(value) for value in (self.offset_east, self.offset_north)):
            self.offset_east = self.offset_north = 0.0
            self.last_publish_ns = None
            self.get_logger().warning('Reset invalid navigation correction offsets')
        message_time = stamp_ns(msg.header.stamp)
        if message_time == 0:
            message_time = self.get_clock().now().nanoseconds
        if self.raw_history and message_time <= self.raw_history[-1][0]:
            if message_time < self.raw_history[-1][0]:
                # Time went backwards (simulator reset, clock change): the
                # history no longer describes the same timeline.
                self.get_logger().warning('Raw odometry time went backwards; '
                                          'clearing history')
                self.raw_history.clear()
            else:
                self.raw_history.pop()
        self.latest_raw = msg
        velocity = enu_velocity(msg)
        if not all(math.isfinite(value) for value in velocity):
            velocity = (0.0, 0.0)
        self.raw_history.append((
            message_time,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            velocity[0], velocity[1]))
        history_ns = int(
            float(self.get_parameter('raw_history_s').value) * 1e9)
        cut = 0
        while (cut < len(self.raw_history) and
               message_time - self.raw_history[cut][0] > history_ns):
            cut += 1
        if cut:
            del self.raw_history[:cut]

        # Fixes that were newer than the previous raw sample are judged now.
        for name in SOURCES:
            state = self._state(name)
            if state.pending is not None:
                self._accept_pending(name, state)

        target_east, target_north, source, confidence, _ = (
            self._target_offset())
        if self.autopilot_absorbs:
            target_east = target_north = 0.0
        if source or self.autopilot_absorbs:
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

        variance, age_s = self._horizontal_variance(msg)
        fused = copy.deepcopy(msg)
        fused.header.frame_id = 'odom'
        fused.pose.pose.position.x += self.offset_east
        fused.pose.pose.position.y += self.offset_north
        fused.pose.covariance[0] = variance
        fused.pose.covariance[7] = variance
        self.output_pub.publish(fused)
        self._publish_fix(fused, variance, age_s)
        self.source_pub.publish(String(data=source or DEAD_RECKONING))
        self.confidence_pub.publish(Float32(data=float(confidence)))

    # ------------------------------------------------------------------
    # Corrections
    # ------------------------------------------------------------------
    def _accept_pending(self, source, state):
        msg = state.pending
        if msg is None or self.latest_raw is None or not state.valid:
            return
        original_stamp = stamp_ns(msg.header.stamp)
        if original_stamp == state.processed_stamp_ns:
            return
        minimum = float(
            self.get_parameter(f'minimum_{source}_confidence').value)
        if not math.isfinite(state.confidence) or not 0 < state.confidence <= 1 or state.confidence < minimum:
            return
        latest_ns = self.raw_history[-1][0] if self.raw_history else 0
        max_age_s = float(self.get_parameter('max_fix_age_s').value)
        message_stamp = original_stamp
        if abs(latest_ns - message_stamp) / 1e9 > max_age_s:
            translated = self._sim_to_wall(message_stamp)
            if translated is not None:
                message_stamp = translated
        tolerance_ns = int(
            float(self.get_parameter('max_time_offset_s').value) * 1e9)
        if 0 < message_stamp - latest_ns <= max_age_s * 1e9 and (
                message_stamp - latest_ns > tolerance_ns):
            # Raw odometry has not reached the fix time yet (low-rate or
            # delayed odometry): judge the fix when a newer sample arrives.
            return
        # Each fix is judged once, whatever the outcome.
        state.processed_stamp_ns = original_stamp
        if self.origin is None:
            self.get_logger().warning(
                f'No navigation origin yet; {source} fix not used '
                '(dead reckoning)', throttle_duration_sec=10.0)
            return
        variance = fix_horizontal_variance(msg)
        if (variance is None or msg.status.status < NavSatStatus.STATUS_FIX
                or not all(math.isfinite(value) for value in (
                    msg.latitude, msg.longitude))
                or not -90 <= msg.latitude <= 90
                or not -180 <= msg.longitude <= 180):
            state.received_ns = 0
            self.get_logger().warning(f'Rejected invalid {source} fix')
            return
        if original_stamp == 0:
            self.get_logger().warning(
                f'Rejected unstamped {source} fix: its time is unknown')
            return
        age_s = (latest_ns - message_stamp) / 1e9
        if age_s > max_age_s:
            self.get_logger().warning(
                f'Rejected {source} fix {age_s:.1f} s older than raw odometry '
                '(stale, or not in the raw odometry clock domain)',
                throttle_duration_sec=5.0)
            return
        raw = self._raw_at(message_stamp)
        if raw is None:
            self.get_logger().warning(
                f'Rejected {source} fix: no raw odometry within '
                f"{self.get_parameter('max_time_offset_s').value} s of its "
                'stamp (simulator vs wall clock?)', throttle_duration_sec=5.0)
            return
        raw_east, raw_north = raw
        fix_east, fix_north = latlon_to_local(
            msg.latitude, msg.longitude, self.origin[0], self.origin[1])
        east = fix_east - raw_east
        north = fix_north - raw_north
        magnitude = math.hypot(east, north)
        if not math.isfinite(magnitude) or magnitude > float(
                self.get_parameter('maximum_correction_m').value):
            self.get_logger().warning(
                f'Rejected {source} correction of {magnitude:.1f} m')
            return
        now_ns = self.get_clock().now().nanoseconds
        state.east = east
        state.north = north
        state.variance = max(1.0, variance)
        state.received_ns = now_ns
        self.last_correction_time = self._now()
        self.last_correction_variance = state.variance
        self._publish_correction_fix(
            msg, fix_east, fix_north, message_stamp, state.variance)

    def _raw_at(self, target_ns):
        """Raw (east, north) at target_ns, interpolated between bracketing
        samples; None if no sample lies within max_time_offset_s."""
        if not self.raw_history or target_ns <= 0:
            return None
        tolerance_ns = int(
            float(self.get_parameter('max_time_offset_s').value) * 1e9)
        max_gap_ns = int(
            float(self.get_parameter('max_interpolation_gap_s').value) * 1e9)
        times = [item[0] for item in self.raw_history]
        index = bisect.bisect_left(times, target_ns)
        if 0 < index < len(times) and times[index - 1] <= target_ns <= times[index]:
            before, after = self.raw_history[index - 1], self.raw_history[index]
            span = after[0] - before[0]
            if span <= max_gap_ns:
                weight = 0.0 if span <= 0 else (target_ns - before[0]) / span
                return (before[1] + weight * (after[1] - before[1]),
                        before[2] + weight * (after[2] - before[2]))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
        nearest = min(candidates, key=lambda i: abs(times[i] - target_ns))
        if abs(times[nearest] - target_ns) > tolerance_ns:
            return None
        return self.raw_history[nearest][1], self.raw_history[nearest][2]

    def _propagate(self, from_ns):
        """Horizontal displacement from from_ns to the latest raw sample by
        integrating raw ENU velocity (trapezoidal)."""
        east = north = 0.0
        previous = None
        for sample in self.raw_history:
            if sample[0] < from_ns:
                previous = sample
                continue
            if previous is None:
                previous = (from_ns,) + sample[1:]
            start = max(previous[0], from_ns)
            dt = (sample[0] - start) / 1e9
            if dt > 0:
                east += 0.5 * (previous[3] + sample[3]) * dt
                north += 0.5 * (previous[4] + sample[4]) * dt
            previous = sample
        return east, north

    def _publish_correction_fix(self, source_msg, fix_east, fix_north,
                                fix_stamp_ns, variance):
        latest_ns = self.raw_history[-1][0]
        delta_east, delta_north = self._propagate(fix_stamp_ns)
        latency_s = max(0.0, (latest_ns - fix_stamp_ns) / 1e9)
        sigma_v = float(self.get_parameter('propagation_velocity_sigma_mps').value)
        variance = variance + (sigma_v * latency_s) ** 2
        lat, lon = local_to_latlon(
            fix_east + delta_east, fix_north + delta_north,
            self.origin[0], self.origin[1])
        fix = NavSatFix()
        fix.header = copy.deepcopy(self.latest_raw.header)
        fix.header.stamp = stamp_from_ns(self.latest_raw.header.stamp, latest_ns)
        fix.header.frame_id = 'map'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = lat, lon
        # DemNav/WildNav fixes carry the prior's altitude, not a measurement.
        fix.altitude = float('nan')
        fix.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, 0.0,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.correction_pub.publish(fix)

    def _target_offset(self):
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(
            float(self.get_parameter('correction_timeout_s').value) * 1e9)
        active = []
        for name in SOURCES:
            state = self._state(name)
            minimum = float(
                self.get_parameter(f'minimum_{name}_confidence').value)
            if (
                    state.valid and 0 < state.confidence <= 1 and state.confidence >= minimum and
                    all(math.isfinite(value) for value in (
                        state.east, state.north, state.confidence, state.variance)) and
                    state.variance > 0 and
                    state.received_ns > 0 and
                    now - state.received_ns <= timeout_ns):
                weight = state.confidence * state.confidence / state.variance
                if math.isfinite(weight) and weight > 0:
                    active.append((name, state, weight))
        if not active:
            return self.offset_east, self.offset_north, '', 0.0, 0.0
        total = sum(item[2] for item in active)
        if not math.isfinite(1.0 / total):
            return self.offset_east, self.offset_north, '', 0.0, 0.0
        east = sum(item[1].east * item[2] for item in active) / total
        north = sum(item[1].north * item[2] for item in active) / total
        source = '+'.join(item[0] for item in active)
        confidence = sum(
            item[1].confidence * item[2] for item in active) / total
        variance = 1.0 / total
        return east, north, source, confidence, variance

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def _horizontal_variance(self, raw):
        """(variance m^2, seconds since the last accepted correction)."""
        if self.last_correction_time is not None:
            base = self.last_correction_variance
            since = self.last_correction_time
        else:
            base = float(self.get_parameter('uncorrected_variance_m2').value)
            since = self.origin_time if self.origin_time is not None else self._now()
        age_s = self._elapsed_s(since)
        drift = float(self.get_parameter('dead_reckoning_drift_mps').value)
        model = base + (drift * age_s) ** 2
        raw_variance = max(raw.pose.covariance[0], raw.pose.covariance[7])
        variance = max(raw_variance, model)
        if not math.isfinite(variance):
            variance = 1e12
        return variance, age_s

    def _publish_fix(self, fused, variance, age_s):
        if self.origin is None:
            return
        lat0, lon0, alt0 = self.origin
        position = fused.pose.pose.position
        fix = NavSatFix()
        fix.header = copy.deepcopy(fused.header)
        fix.header.frame_id = 'map'
        limit = float(self.get_parameter('dead_reckoning_limit_s').value)
        fix.status.status = (
            NavSatStatus.STATUS_FIX if age_s <= limit
            else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = local_to_latlon(
            position.x, position.y, lat0, lon0)
        # Same datum as the origin (ellipsoid for MAVROS).
        fix.altitude = alt0 + position.z
        vertical = fused.pose.covariance[14]
        fix.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, vertical if vertical > 0 else variance,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

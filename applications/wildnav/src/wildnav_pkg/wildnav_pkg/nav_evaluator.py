#!/usr/bin/env python3
"""Compare navigation estimates against a truth GPS stream.

In SITL the simulated GPS is ground truth (plus small modelled noise), so
subscribing to it alongside the WildNav/DemNav fixes and the fused odometry
turns a simulation run into a measurable accuracy report instead of a pile
of logs. The node keeps running horizontal-error statistics per source,
logs a summary line periodically, and publishes the same summary as JSON on
``summary_topic`` for dashboards and smoke tests.

With ``assert_after_s`` set, the node evaluates pass/fail criteria after
that long and exits with a nonzero code on failure, which is what
``tools/sitl_smoke_test.sh`` uses to gate CI/nightly runs.
"""
import json
import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from synthetic_world import latlon_to_local, local_to_latlon


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SourceStats:
    def __init__(self):
        self.errors = deque(maxlen=10000)
        self.count = 0
        self.max_error = 0.0

    def add(self, error_m):
        self.errors.append(error_m)
        self.count += 1
        self.max_error = max(self.max_error, error_m)

    def summary(self):
        if not self.errors:
            return {'fixes': self.count}
        ordered = sorted(self.errors)
        rmse = math.sqrt(sum(e * e for e in ordered) / len(ordered))
        p95_rank = max(0, math.ceil(0.95 * len(ordered)) - 1)
        return {
            'fixes': self.count,
            'rmse_m': round(rmse, 2),
            'mean_m': round(sum(ordered) / len(ordered), 2),
            'p95_m': round(ordered[p95_rank], 2),
            'max_m': round(self.max_error, 2),
        }


class NavEvaluator(Node):
    def __init__(self):
        super().__init__('nav_evaluator')
        self.declare_parameter('truth_gps_topic',
                               '/plane0/sensor_measurements/gps')
        self.declare_parameter('wildnav_fix_topic', '/wildnav/fix')
        self.declare_parameter('demnav_fix_topic', '/demnav/fix')
        self.declare_parameter('fused_odom_topic', '/navigation/odometry')
        self.declare_parameter('initial_lat', 0.0)
        self.declare_parameter('initial_lon', 0.0)
        self.declare_parameter('summary_topic', '/nav_eval/summary')
        self.declare_parameter('report_period_s', 30.0)
        self.declare_parameter('match_tolerance_s', 0.5)
        self.declare_parameter('fused_sample_period_s', 1.0)
        # Smoke-test mode: after assert_after_s, check the criteria below
        # and exit (nonzero on failure). 0 runs forever.
        self.declare_parameter('assert_after_s', 0.0)
        self.declare_parameter('assert_sources', 'wildnav,demnav')
        self.declare_parameter('max_rmse_m', 25.0)
        self.declare_parameter('min_fixes', 3)

        self.origin_lat = float(self.get_parameter('initial_lat').value)
        self.origin_lon = float(self.get_parameter('initial_lon').value)
        self.tolerance_ns = int(
            float(self.get_parameter('match_tolerance_s').value) * 1e9)
        self.truth = deque(maxlen=1200)
        self.stats = {
            'wildnav': SourceStats(),
            'demnav': SourceStats(),
            'fused': SourceStats(),
        }
        self.start_ns = None
        self.last_fused_ns = 0
        self.exit_code = 0

        self.create_subscription(
            NavSatFix, str(self.get_parameter('truth_gps_topic').value),
            self._on_truth, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('wildnav_fix_topic').value),
            lambda m: self._on_fix('wildnav', m), qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('demnav_fix_topic').value),
            lambda m: self._on_fix('demnav', m), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('fused_odom_topic').value),
            self._on_fused, qos_profile_sensor_data)
        self.summary_pub = self.create_publisher(
            String, str(self.get_parameter('summary_topic').value), 10)

        period = max(1.0, float(self.get_parameter('report_period_s').value))
        self.create_timer(period, self._report)
        assert_after = float(self.get_parameter('assert_after_s').value)
        if assert_after > 0.0:
            self.assert_timer = self.create_timer(assert_after, self._assert)
        self.get_logger().info('Navigation evaluator running')

    def _on_truth(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        t = _stamp_ns(msg.header.stamp)
        if self.start_ns is None:
            self.start_ns = t
        self.truth.append((t, float(msg.latitude), float(msg.longitude)))

    def _error_vs_truth(self, t_ns, lat, lon):
        if not self.truth:
            return None
        nearest = min(self.truth, key=lambda s: abs(s[0] - t_ns))
        if abs(nearest[0] - t_ns) > self.tolerance_ns:
            return None
        east, north = latlon_to_local(lat, lon, nearest[1], nearest[2])
        return math.hypot(east, north)

    def _on_fix(self, source, msg: NavSatFix):
        error = self._error_vs_truth(
            _stamp_ns(msg.header.stamp),
            float(msg.latitude), float(msg.longitude))
        if error is not None:
            self.stats[source].add(error)

    def _on_fused(self, msg: Odometry):
        t = _stamp_ns(msg.header.stamp)
        sample_ns = int(
            float(self.get_parameter('fused_sample_period_s').value) * 1e9)
        if t - self.last_fused_ns < sample_ns:
            return
        if self.origin_lat == 0.0 and self.origin_lon == 0.0:
            return
        self.last_fused_ns = t
        lat, lon = local_to_latlon(
            float(msg.pose.pose.position.x), float(msg.pose.pose.position.y),
            self.origin_lat, self.origin_lon)
        error = self._error_vs_truth(t, lat, lon)
        if error is not None:
            self.stats['fused'].add(error)

    def _summary(self):
        elapsed = 0.0
        if self.start_ns is not None and self.truth:
            elapsed = (self.truth[-1][0] - self.start_ns) / 1e9
        summary = {'elapsed_s': round(elapsed, 1), 'sources': {}}
        for name, stats in self.stats.items():
            entry = stats.summary()
            if elapsed > 0.0:
                entry['rate_hz'] = round(entry['fixes'] / elapsed, 3)
            summary['sources'][name] = entry
        return summary

    def _report(self):
        summary = self._summary()
        payload = json.dumps(summary)
        self.summary_pub.publish(String(data=payload))
        self.get_logger().info(f'NAV-EVAL {payload}')

    def _assert(self):
        self.assert_timer.cancel()
        summary = self._summary()
        max_rmse = float(self.get_parameter('max_rmse_m').value)
        min_fixes = int(self.get_parameter('min_fixes').value)
        sources = [s.strip() for s in
                   str(self.get_parameter('assert_sources').value).split(',')
                   if s.strip()]
        failures = []
        for name in sources:
            entry = summary['sources'].get(name, {})
            if entry.get('fixes', 0) < min_fixes:
                failures.append(
                    f'{name}: {entry.get("fixes", 0)} fixes < {min_fixes}')
            elif entry.get('rmse_m', float('inf')) > max_rmse:
                failures.append(
                    f'{name}: rmse {entry.get("rmse_m")} m > {max_rmse} m')
        self._report()
        if failures:
            self.exit_code = 1
            self.get_logger().error('NAV-EVAL RESULT: FAIL — ' +
                                    '; '.join(failures))
        else:
            self.get_logger().info('NAV-EVAL RESULT: PASS')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

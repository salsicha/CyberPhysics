#!/usr/bin/env python3
"""Record an Aerodrone simulation's navigation for the GPS-denied test.

The ``gps_denial_recorder`` service of the ``denial`` profile of the
Gazebo and Isaac compositions (tools/gps_denied_sim_test.sh and
tools/gps_denied_sim_test.md). Writes one compact JSON line per sample to
``--output``, each with its arrival wall time ``rx`` (and ``t``, the header
stamp, where the message has one), for tools/gps_denied_sim_scorer.py:

- ``meta``: the stack, the topics and the projection origin
  (INITIAL_LAT/INITIAL_LON, the origin of the simulators' ``earth`` frame);
- ``clock``: simulation time (``/clock``) every ``--clock-period-s`` of wall
  time, to put wall-time arrivals on the simulation clock;
- ``truth``: simulator ground truth, east/north/up metres in that frame;
- ``autopilot``: ArduPilot's own position (MAVROS ``global_position/global``);
- ``fused``: ``/navigation/fix`` with its horizontal covariance;
- ``correction``, ``demnav``, ``wildnav``: every ``/navigation/correction_fix``,
  ``/demnav/fix`` and ``/wildnav/fix`` with its horizontal covariance;
- ``source``, ``state``, ``gps1``, ``setpoint``: changes of the correction
  source, the autopilot mode/arming, GPS 1's fix type and the MAVROS position
  setpoint (the mission's GoTo targets);
- ``text``: autopilot status texts; ``origin``: the EKF origin.

truth is thinned to one row per ``--truth-period-s`` and autopilot/fused to
one per ``--period-s`` of simulation time (wall time before ``/clock``
arrives), which keeps a 40-minute Isaac run to a few megabytes.

The row builders are plain Python; ROS is imported only by run().
"""

import argparse
import json
import signal
import sys
import time


def _r(value, digits):
    return round(float(value), digits)


def horizontal_covariance(msg):
    """[east-east, east-north, north-north] of a NavSatFix (ENU, row-major)."""
    cov = msg.position_covariance
    return [_r(cov[0], 4), _r(cov[1], 4), _r(cov[4], 4)]


def fix_row(kind, msg, rx):
    stamp = msg.header.stamp
    return {'k': kind, 'rx': _r(rx, 3), 't': _r(stamp.sec + stamp.nanosec * 1e-9, 4),
            'lat': _r(msg.latitude, 9), 'lon': _r(msg.longitude, 9),
            'alt': _r(msg.altitude, 2), 'status': int(msg.status.status),
            'cov': horizontal_covariance(msg)}


def truth_row(msg, rx):
    """Ground truth from an Odometry (Isaac) or a PoseStamped (Gazebo)."""
    pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
    stamp = msg.header.stamp
    p = pose.position
    return {'k': 'truth', 'rx': _r(rx, 3), 't': _r(stamp.sec + stamp.nanosec * 1e-9, 4),
            'e': _r(p.x, 3), 'n': _r(p.y, 3), 'u': _r(p.z, 3)}


class Thinner:
    """Keeps one sample per period of simulation time, or of wall time
    until the simulation clock is known."""

    def __init__(self, period_s):
        self.period_s = float(period_s)
        self.last = None

    def keep(self, sim, rx):
        now = rx if sim is None else sim
        clock = 'wall' if sim is None else 'sim'
        if (self.last is not None and self.last[0] == clock
                and 0.0 <= now - self.last[1] < 0.9 * self.period_s):
            return False
        self.last = (clock, now)
        return True


class ChangeFilter:
    """Passes a value only when it differs from the previous one."""

    def __init__(self):
        self.last = object()

    def changed(self, value):
        if value == self.last:
            return False
        self.last = value
        return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', default='/out/recording.jsonl')
    parser.add_argument('--stack', default='')
    parser.add_argument('--truth-topic', default='/aerodrone/simulation/ground_truth/odom')
    parser.add_argument('--truth-type', choices=('odometry', 'pose'), default='odometry')
    parser.add_argument('--origin-lat', type=float, required=True)
    parser.add_argument('--origin-lon', type=float, required=True)
    parser.add_argument('--mavros-namespace', default='/mavros')
    parser.add_argument('--clock-topic', default='/clock')
    parser.add_argument('--truth-period-s', type=float, default=0.1)
    parser.add_argument('--period-s', type=float, default=0.2)
    parser.add_argument('--clock-period-s', type=float, default=0.5)
    return parser.parse_args(argv)


def record(args, stream):
    import rclpy
    from geographic_msgs.msg import GeoPointStamped
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import GPSRAW, State, StatusText
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String

    ns = args.mavros_namespace
    fixes = {'correction': '/navigation/correction_fix', 'demnav': '/demnav/fix',
             'wildnav': '/wildnav/fix'}

    def write(row):
        stream.write(json.dumps(row, separators=(',', ':')) + '\n')

    write({'k': 'meta', 'rx': _r(time.time(), 3), 'stack': args.stack,
           'origin_lat': args.origin_lat, 'origin_lon': args.origin_lon,
           'truth_topic': args.truth_topic, 'fused_topic': '/navigation/fix',
           'autopilot_topic': f'{ns}/global_position/global', 'fix_topics': fixes})

    class Recorder(Node):
        def __init__(self):
            super().__init__('gps_denial_recorder')
            sensor = qos_profile_sensor_data
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.sim = None
            self.last_clock_row = 0.0
            self.thin = {'truth': Thinner(args.truth_period_s),
                         'autopilot': Thinner(args.period_s), 'fused': Thinner(args.period_s)}
            self.changes = {name: ChangeFilter()
                            for name in ('source', 'state', 'gps1', 'setpoint')}
            self.create_subscription(Clock, args.clock_topic, self._on_clock, sensor)
            truth_type = Odometry if args.truth_type == 'odometry' else PoseStamped
            self.create_subscription(truth_type, args.truth_topic, self._on_truth, sensor)
            self.create_subscription(NavSatFix, f'{ns}/global_position/global',
                                     lambda m: self._on_thinned_fix('autopilot', m), sensor)
            self.create_subscription(NavSatFix, '/navigation/fix',
                                     lambda m: self._on_thinned_fix('fused', m), sensor)
            for kind, topic in fixes.items():
                self.create_subscription(
                    NavSatFix, topic, lambda m, k=kind: write(fix_row(k, m, time.time())), 50)
            self.create_subscription(String, '/navigation/correction_source',
                                     lambda m: self._on_change('source', {'v': m.data}), 10)
            self.create_subscription(
                State, f'{ns}/state',
                lambda m: self._on_change('state', {'mode': m.mode, 'armed': bool(m.armed)}), 10)
            self.create_subscription(
                GPSRAW, f'{ns}/gpsstatus/gps1/raw',
                lambda m: self._on_change('gps1', {'fix_type': int(m.fix_type)}), sensor)
            self.create_subscription(
                PoseStamped, f'{ns}/setpoint_position/local',
                lambda m: self._on_change('setpoint', {
                    'x': _r(m.pose.position.x, 1), 'y': _r(m.pose.position.y, 1),
                    'z': _r(m.pose.position.z, 1)}), 10)
            self.create_subscription(
                StatusText, f'{ns}/statustext/recv',
                lambda m: write({'k': 'text', 'rx': _r(time.time(), 3), 'text': m.text}),
                sensor)
            self.create_subscription(
                GeoPointStamped, '/navigation/origin',
                lambda m: write({'k': 'origin', 'rx': _r(time.time(), 3),
                                 'lat': m.position.latitude, 'lon': m.position.longitude,
                                 'alt': m.position.altitude}), latched)

        def _on_clock(self, msg):
            self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9
            rx = time.time()
            if rx - self.last_clock_row >= args.clock_period_s:
                self.last_clock_row = rx
                write({'k': 'clock', 'rx': _r(rx, 3), 'sim': _r(self.sim, 4)})

        def _on_truth(self, msg):
            rx = time.time()
            if self.thin['truth'].keep(self.sim, rx):
                write(truth_row(msg, rx))

        def _on_thinned_fix(self, kind, msg):
            rx = time.time()
            if self.thin[kind].keep(self.sim, rx):
                write(fix_row(kind, msg, rx))

        def _on_change(self, kind, fields):
            if self.changes[kind].changed(fields):
                write({'k': kind, 'rx': _r(time.time(), 3), **fields})

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass  # stopped with the stack: every row written so far is complete
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def run(args):
    with open(args.output, 'a', buffering=1) as stream:
        return record(args, stream)


def main(argv=None):
    args = parse_args(argv)
    # PID 1 in its container: without a handler SIGTERM would be ignored.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    return run(args)


if __name__ == '__main__':
    sys.exit(main())

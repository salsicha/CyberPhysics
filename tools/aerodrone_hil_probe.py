#!/usr/bin/env python3
"""ROS topic probe for hardware-in-the-loop runs (tools/aerodrone_hil_test.md).

Subscribes to topics as serialized messages (no deserialization: cheap on
the Pi) for a window and reports, per topic, what arrived on this host:
messages, rate, bytes per second and mean size, the longest gap, and for
stamped messages (std_msgs/Header first, or rosgraph_msgs/Clock) the age at
arrival, wall clock minus header stamp (minus --clock-offset-s, this host's
clock minus the stamping host's). With --timesync it also records MAVROS's
TIMESYNC round trips (/mavros/timesync_status, the MAVLink round trip
between MAVROS and the autopilot).

  python3 aerodrone_hil_probe.py --duration-s 60 --output probe.json \
      /oak1/relative_depth /tracking/detections ...

Run it on the host whose arrivals are of interest. Subscribing on a host
where no node consumes a topic pulls that topic across the network: probe
only topics a node on that host already takes.
"""
import argparse
import json
import math
import struct
import sys
import threading
import time

STAMPED_FIRST_FIELDS = {('header', 'std_msgs/Header'), ('clock', 'builtin_interfaces/Time')}


def cdr_stamp(raw):
    """(sec, nanosec) of the leading builtin_interfaces/Time of a CDR
    message (after the 4-byte encapsulation header), or None."""
    if len(raw) < 12:
        return None
    little = raw[1] & 1  # CDR_LE / PL_CDR_LE
    sec, nanosec = struct.unpack('<iI' if little else '>iI', bytes(raw[4:12]))
    return sec, nanosec


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


class TopicStats:
    def __init__(self, stamped):
        self.stamped = stamped
        self.count = 0
        self.bytes = 0
        self.first = None
        self.last = None
        self.max_gap = 0.0
        self.ages = []
        self.sim_stamps = 0

    def add(self, rx, raw, offset_s=0.0):
        if self.last is not None:
            self.max_gap = max(self.max_gap, rx - self.last)
        if self.first is None:
            self.first = rx
        self.last = rx
        self.count += 1
        self.bytes += len(raw)
        if self.stamped:
            stamp = cdr_stamp(raw)
            if stamp is not None:
                t = stamp[0] + stamp[1] * 1e-9
                if abs(rx - t) > 1e6:
                    self.sim_stamps += 1  # simulation time: no age
                else:
                    self.ages.append(rx - offset_s - t)

    def summary(self, window_s):
        out = {'count': self.count, 'rate_hz': round(self.count / window_s, 3),
               'bytes_per_s': round(self.bytes / window_s, 1),
               'mean_bytes': round(self.bytes / self.count, 1) if self.count else None,
               'max_gap_s': round(self.max_gap, 3) if self.count > 1 else None}
        if self.ages:
            out['age_ms'] = {'median': round(1e3 * percentile(self.ages, 0.5), 1),
                             'p95': round(1e3 * percentile(self.ages, 0.95), 1),
                             'max': round(1e3 * max(self.ages), 1),
                             'min': round(1e3 * min(self.ages), 1), 'n': len(self.ages)}
        if self.sim_stamps:
            out['sim_stamped'] = self.sim_stamps
        return out


def main(argv=None):  # pragma: no cover - ROS runtime
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('topics', nargs='*')
    parser.add_argument('--duration-s', type=float, default=60.0)
    parser.add_argument('--discover-s', type=float, default=20.0,
                        help='wait this long for the topics to appear')
    parser.add_argument('--clock-offset-s', type=float, default=0.0,
                        help="this host's clock minus the stamping host's")
    parser.add_argument('--timesync', action='store_true')
    parser.add_argument('--label', default='')
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('hil_probe')
    deadline = time.time() + args.discover_s
    types = {}
    while time.time() < deadline:
        known = dict(node.get_topic_names_and_types())
        types = {t: known[t][0] for t in args.topics if known.get(t)}
        if len(types) == len(args.topics):
            break
        rclpy.spin_once(node, timeout_sec=0.5)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    lock = threading.Lock()
    stats = {}
    for topic, type_name in types.items():
        msg_class = get_message(type_name)
        fields = list(msg_class.get_fields_and_field_types().items())
        stamped = bool(fields) and tuple(fields[0]) in STAMPED_FIRST_FIELDS
        stats[topic] = TopicStats(stamped)

        def callback(raw, topic=topic):
            rx = time.time()
            with lock:
                stats[topic].add(rx, raw, args.clock_offset_s)
        node.create_subscription(msg_class, topic, callback, qos, raw=True)
    rtts = []
    if args.timesync:
        from mavros_msgs.msg import TimesyncStatus
        node.create_subscription(TimesyncStatus, '/mavros/timesync_status',
                                 lambda m: rtts.append(float(m.round_trip_time_ms)), 10)
    start = time.time()
    while time.time() - start < args.duration_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    window = time.time() - start
    with lock:
        report = {'label': args.label, 'start': round(start, 3), 'window_s': round(window, 2),
                  'clock_offset_s': args.clock_offset_s,
                  'missing': sorted(set(args.topics) - set(types)),
                  'types': types,
                  'topics': {topic: s.summary(window) for topic, s in stats.items()}}
    if args.timesync:
        finite = [r for r in rtts if math.isfinite(r) and r > 0]
        report['mavlink_rtt_ms'] = {
            'n': len(finite),
            'median': round(percentile(finite, 0.5), 2) if finite else None,
            'p95': round(percentile(finite, 0.95), 2) if finite else None,
            'max': round(max(finite), 2) if finite else None}
    report['total_bytes_per_s'] = round(sum(t['bytes_per_s'] for t in report['topics'].values()), 1)
    with open(args.output, 'w', encoding='utf-8') as out:
        json.dump(report, out, indent=1)
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

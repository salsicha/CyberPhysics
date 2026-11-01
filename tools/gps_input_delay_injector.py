#!/usr/bin/env python3
"""Late and bursty GPS-denied corrections, for tests (tools/gps_input_delay.yaml).

Sits on both sides of the navigation bridge and delays one of them the way a
slow companion computer does (tools/aerodrone_hil_test.md: on the throttled
Raspberry Pi 5 the corrections were 0.3 s to several seconds late, in
bursts):

- ``before_bridge``: ``/navigation/correction_fix`` (sensor_msgs/NavSatFix)
  reaches the bridge late, the companion's own lag (the fusion, DemNav and
  WildNav behind a saturated CPU);
- ``after_bridge``: the bridge's GPS_INPUT (mavros_msgs/GPSINPUT) reaches
  MAVROS late, a slow link to the autopilot.

The other side is relayed at once. Every message keeps its header and stamp;
only its arrival moves: ``delay_s`` plus a uniform ``jitter_s``, first in
first out (a lagging executor does not reorder); with ``burst_s`` > 0
everything is held and released together at the next multiple of
``burst_s`` after its due time (a stalled pipeline catching up); ``drop`` is
the share never released. Delays are on this node's clock (wall time, like
the bridge's).
"""
from __future__ import annotations

import collections
import math
import random

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

STAGES = ('before_bridge', 'after_bridge')


class DelayLine:
    """FIFO release times for delay + jitter, optional bursts and drops."""

    def __init__(self, delay_s=0.0, jitter_s=0.0, burst_s=0.0, drop=0.0, seed=1):
        self.delay_s = max(0.0, float(delay_s))
        self.jitter_s = max(0.0, float(jitter_s))
        self.burst_s = max(0.0, float(burst_s))
        self.drop = min(1.0, max(0.0, float(drop)))
        self.rng = random.Random(int(seed))
        self.queue = collections.deque()
        self.last_due = -math.inf
        self.received = self.released = self.dropped = 0

    def push(self, now_s, item):
        self.received += 1
        if self.drop and self.rng.random() < self.drop:
            self.dropped += 1
            return None
        due = max(self.last_due, now_s + self.delay_s + self.rng.uniform(0.0, self.jitter_s))
        self.last_due = due
        if self.burst_s > 0:
            due = (math.floor(due / self.burst_s) + 1) * self.burst_s
        self.queue.append((due, item))
        return due

    def pop_due(self, now_s):
        out = []
        while self.queue and self.queue[0][0] <= now_s:
            out.append(self.queue.popleft()[1])
        self.released += len(out)
        return out


class GpsInputDelayInjector(Node):
    def __init__(self):
        super().__init__('gps_input_delay_injector')
        defaults = {
            'stage': 'before_bridge',
            'correction_topic': '/navigation/correction_fix',
            'bridge_correction_topic': '/navigation/correction_fix_in',
            'bridge_gps_input_topic': '/navigation/gps_input_out',
            'mavros_gps_input_topic': '/mavros/gps_input/gps_input',
            'delay_s': 0.0,
            'jitter_s': 0.0,
            'burst_s': 0.0,
            'drop': 0.0,
            'seed': 1,
        }
        for name, value in defaults.items():
            # Numbers from the environment may come as integers ("2").
            self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=True))
        p = {name: self.get_parameter(name).value for name in defaults}
        if p['stage'] not in STAGES:
            raise ValueError(f"stage must be one of {STAGES}, not {p['stage']!r}")
        from mavros_msgs.msg import GPSINPUT
        from sensor_msgs.msg import NavSatFix
        self.lines = []
        for stage, kind, source, target in (
                ('before_bridge', NavSatFix, p['correction_topic'], p['bridge_correction_topic']),
                ('after_bridge', GPSINPUT, p['bridge_gps_input_topic'], p['mavros_gps_input_topic'])):
            delayed = stage == p['stage']
            line = DelayLine(p['delay_s'], p['jitter_s'], p['burst_s'], p['drop'], p['seed']) \
                if delayed else DelayLine()
            publisher = self.create_publisher(kind, target, 10)
            self.lines.append((line, publisher))
            self.create_subscription(
                kind, source, lambda msg, line=line: line.push(self._now_s(), msg), 10)
            self.get_logger().warn(
                f'{source} -> {target}: ' + (
                    f"{p['delay_s']:g} s + U(0, {p['jitter_s']:g}) s, bursts every "
                    f"{p['burst_s']:g} s, drop {p['drop']:g}" if delayed else 'relayed'))
        self.create_timer(0.01, self._release)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _release(self):
        now_s = self._now_s()
        for line, publisher in self.lines:
            for msg in line.pop_due(now_s):
                publisher.publish(msg)
                if line.released % 100 == 0:
                    self.get_logger().info(
                        f'{publisher.topic_name}: {line.received} received, '
                        f'{line.released} released, {line.dropped} dropped, '
                        f'{len(line.queue)} held')


def main():
    rclpy.init()
    node = GpsInputDelayInjector()
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

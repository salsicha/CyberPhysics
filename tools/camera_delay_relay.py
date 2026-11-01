#!/usr/bin/env python3
"""A slower camera, for tests: republishes an image topic and its
camera_info a fixed delay after they arrive, headers unchanged (capture
stamps), as a camera pipeline that takes that long would deliver them.

tools/camera_delay.yaml puts it in front of the Gazebo stack's tracking
detector, so the detections, and the tracker's input, reach it late with
their capture stamps (tools/aerodrone_hil_test.md, section 3: the camera's
age at the tracker is 0.14 s all-local, 0.18-0.27 s in HIL, and the real
OAK-1's detections arrive 0.16 s after capture, p95 0.31 s).

Parameters: in_image, in_info, out_image, out_info (topics), delay_s
(seconds of wall time, the relay's own clock), jitter_s (uniform, added),
stamp_offset_s (added to every header stamp: a camera clock off the
odometry's by that much; 0 keeps the capture stamps). The order of the
frames is kept.
"""
import collections
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DelayQueue:
    """Messages released delay_s (+ uniform jitter_s) after they were
    queued, in order: a later message never overtakes an earlier one."""

    def __init__(self, delay_s, jitter_s=0.0, rng=None):
        self.delay_s = max(0.0, float(delay_s))
        self.jitter_s = max(0.0, float(jitter_s))
        self.rng = rng or random.Random(1)
        self.queue = collections.deque()
        self.last_due = -float('inf')

    def put(self, now, item):
        due = now + self.delay_s + self.rng.uniform(0.0, self.jitter_s)
        self.last_due = max(self.last_due, due)
        self.queue.append((self.last_due, item))

    def due(self, now):
        out = []
        while self.queue and self.queue[0][0] <= now:
            out.append(self.queue.popleft()[1])
        return out


class CameraDelayRelay(Node):
    def __init__(self):
        super().__init__('camera_delay_relay')
        param = {name: self.declare_parameter(name, default).value for name, default in (
            ('in_image', '/aerodrone/sensor_measurements/downward_rgb/image_raw'),
            ('in_info', '/aerodrone/sensor_measurements/downward_rgb/camera_info'),
            ('out_image', '/camera_delay/image_raw'),
            ('out_info', '/camera_delay/camera_info'),
            ('delay_s', 0.15), ('jitter_s', 0.0), ('stamp_offset_s', 0.0))}
        self.offset_ns = int(round(float(param['stamp_offset_s']) * 1e9))
        self.queue = DelayQueue(param['delay_s'], param['jitter_s'])
        self.image_pub = self.create_publisher(Image, param['out_image'], qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, param['out_info'], qos_profile_sensor_data)
        self.create_subscription(Image, param['in_image'],
                                 lambda msg: self.queue.put(time.monotonic(), (self.image_pub, msg)),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, param['in_info'],
                                 lambda msg: self.queue.put(time.monotonic(), (self.info_pub, msg)),
                                 qos_profile_sensor_data)
        self.create_timer(0.005, self._release)
        self.get_logger().info(
            f"{param['in_image']} -> {param['out_image']} {param['delay_s']:.3f} s later"
            f" (+ up to {param['jitter_s']:.3f} s), stamps {param['stamp_offset_s']:+.3f} s")

    def _release(self):
        for publisher, msg in self.queue.due(time.monotonic()):
            if self.offset_ns:  # the message is this relay's own: shifted in place
                msg.header.stamp = shifted(msg.header.stamp, self.offset_ns)
            publisher.publish(msg)


def shifted(stamp, offset_ns):
    """A new stamp of stamp's type, offset_ns later (earlier if negative)."""
    total = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) + int(offset_ns)
    out = type(stamp)()
    out.sec, out.nanosec = divmod(max(0, total), 1_000_000_000)
    return out


def main():
    rclpy.init()
    node = CameraDelayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

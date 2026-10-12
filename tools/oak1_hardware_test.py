#!/usr/bin/env python3
"""Measure the running OAK-1 ROS pipeline; no scene or target is required.

Run inside the OAK image on its ROS domain with --tracking to exercise both
networks. Reports delivery gaps, not inferred sensor drops: MegaDepth and
YOLO intentionally process different subsets of the 20 Hz sensor stream.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import Detection2DArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--tracking', action='store_true')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('oak1_hardware_test')
    samples, details, subscriptions = {}, {}, []
    start = time.monotonic() + args.warmup

    def subscribe(topic, kind):
        samples[topic] = []

        def receive(msg):
            now = time.monotonic()
            if now < start:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            samples[topic].append((now, stamp, time.time() - stamp))
            if isinstance(msg, Image):
                info = {'width': msg.width, 'height': msg.height,
                        'encoding': msg.encoding, 'bytes': len(msg.data)}
                if msg.encoding == '32FC1':
                    data = np.frombuffer(bytes(msg.data), dtype='>f4' if msg.is_bigendian else '<f4')
                    info['finite_fraction'] = float(np.isfinite(data).mean())
                else:
                    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                    info['pixel_mean'] = float(data.mean())
                    info['pixel_std'] = float(data.std())
                details[topic] = info
            elif isinstance(msg, CameraInfo):
                details[topic] = {'width': msg.width, 'height': msg.height,
                                  'fx': msg.k[0], 'fy': msg.k[4],
                                  'calibration_valid': bool(msg.k[0] > 0 and msg.k[4] > 0)}
            elif isinstance(msg, Detection2DArray):
                details[topic] = {'last_detection_count': len(msg.detections)}
            else:
                details[topic] = {'format': msg.format, 'bytes': len(msg.data)}

        subscriptions.append(node.create_subscription(kind, topic, receive, qos_profile_sensor_data))

    for topic, kind in [('/oak1/image_raw', Image), ('/oak1/relative_depth', Image),
                        ('/oak1/camera_info', CameraInfo), ('/oak1/image_highres', Image)]:
        subscribe(topic, kind)
    if args.tracking:
        for topic, kind in [('/tracking/detections', Detection2DArray),
                            ('/tracking/image/compressed', CompressedImage),
                            ('/tracking/camera_info', CameraInfo)]:
            subscribe(topic, kind)
    try:
        while rclpy.ok() and time.monotonic() < start + args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = {'tracking_subscribed': args.tracking, 'seconds': args.seconds, 'topics': {}}
        for topic, rows in samples.items():
            gaps = np.diff([r[0] for r in rows])
            report['topics'][topic] = dict(
                count=len(rows), hz=len(rows) / args.seconds,
                unique_stamps=len({r[1] for r in rows}),
                gap_p95_s=float(np.percentile(gaps, 95)) if len(gaps) else None,
                gap_max_s=float(max(gaps)) if len(gaps) else None,
                age_p95_s=float(np.percentile([r[2] for r in rows], 95)) if rows else None,
                **details.get(topic, {}))
        if args.tracking:
            a = {r[1] for r in samples['/tracking/detections']}
            b = {r[1] for r in samples['/tracking/image/compressed']}
            c = {r[1] for r in samples['/tracking/camera_info']}
            report['tracking_paired_stamps'] = len(a & b & c)
            report['tracking_unpaired_stamps'] = len((a | b | c) - (a & b & c))
        report['passed'] = (
            all(rows for rows in samples.values())
            and report['topics']['/oak1/relative_depth']['hz'] >= (2.0 if args.tracking else 3.0)
            and report['topics']['/oak1/relative_depth'].get('finite_fraction', 0) > .99
            and report['topics']['/oak1/camera_info'].get('calibration_valid', False)
            and report['topics']['/oak1/image_highres']['hz'] >= .4
            and report['topics']['/oak1/image_highres']['gap_max_s'] <= 5
            and (not args.tracking or report['topics']['/tracking/detections']['hz'] >= 4.0))
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

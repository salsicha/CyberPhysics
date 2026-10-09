#!/usr/bin/env python3
"""ROS 2 node: room-level location from WiFi fingerprints (vendored whereami).

Every `scan_period_s` it scans WiFi access points, runs whereami's trained
model (model.pkl in `model_dir`, reloaded when it changes) and publishes:

    location_topic       std_msgs/String   most likely location label
    confidence_topic     std_msgs/Float32  its probability (0-1)
    probabilities_topic  std_msgs/String   JSON {label: probability}, as
                                           `whereami predict_proba` prints

All three are reliable and transient-local (depth 1): a late subscriber
gets the current location. Nothing is published until a model exists; the
node logs how to train one. Scanner defaults come from the same
WHEREAMI_SCANNER / WHEREAMI_DEVICE / WHEREAMI_SCAN_FILE /
WHEREAMI_NMCLI_RESCAN environment variables the training CLI uses.
"""
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from whereami_localizer import (OK, SCANNERS, Localizer, ModelStore,
                                make_scanner)

# Same location whereami's `learn` writes to (~/.whereami of the user).
DEFAULT_MODEL_DIR = os.path.expanduser('~/.whereami')
REPEAT_LOG_S = 60.0


class WhereamiNode(Node):

    def __init__(self, localizer=None):
        super().__init__('whereami')
        env = os.environ.get
        self.declare_parameter('scanner', env('WHEREAMI_SCANNER', 'auto'))
        self.declare_parameter('device', env('WHEREAMI_DEVICE', ''))
        self.declare_parameter('scan_file', env('WHEREAMI_SCAN_FILE', ''))
        self.declare_parameter('nmcli_rescan', env('WHEREAMI_NMCLI_RESCAN', 'auto'))
        self.declare_parameter('model_dir', DEFAULT_MODEL_DIR)
        self.declare_parameter('scan_period_s', 10.0)
        self.declare_parameter('scan_timeout_s', 20.0)
        self.declare_parameter('location_topic', '/whereami/location')
        self.declare_parameter('confidence_topic', '/whereami/confidence')
        self.declare_parameter('probabilities_topic', '/whereami/probabilities')

        def param(name):
            return self.get_parameter(name).value

        scanner = str(param('scanner'))
        if scanner not in SCANNERS:
            raise ValueError(f'Unknown scanner {scanner!r}; expected one of {SCANNERS}')
        if localizer is None:
            settings = dict(backend=scanner, device=str(param('device')),
                            scan_file=str(param('scan_file')),
                            timeout_s=float(param('scan_timeout_s')),
                            rescan=str(param('nmcli_rescan')))
            localizer = Localizer(lambda: make_scanner(**settings),
                                  ModelStore(str(param('model_dir'))))
        self.localizer = localizer

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.location_pub = self.create_publisher(String, param('location_topic'), qos)
        self.confidence_pub = self.create_publisher(Float32, param('confidence_topic'), qos)
        self.probabilities_pub = self.create_publisher(
            String, param('probabilities_topic'), qos)

        self._last_status = None
        self._last_label = None
        self._last_log_time = -math.inf
        self.get_logger().info(
            f'whereami: scanner {scanner}, model '
            f'{self.localizer.model_store.path}, every {float(param("scan_period_s")):.1f} s')
        self.create_timer(float(param('scan_period_s')), self.tick)
        self.tick()  # report a missing model or scanner right away

    def tick(self):
        result = self.localizer.step()
        self._log(result)
        if result.status != OK:
            return
        self.location_pub.publish(String(data=result.label))
        self.confidence_pub.publish(Float32(data=float(result.confidence)))
        self.probabilities_pub.publish(
            String(data=json.dumps(result.probabilities, sort_keys=True)))

    def _log(self, result):
        logger = self.get_logger()
        if result.model_reloaded:
            store = self.localizer.model_store
            logger.info(f'Loaded {store.path}: locations {", ".join(store.labels)}')
        now = time.monotonic()
        if result.status == OK:
            if result.label != self._last_label or self._last_status != OK:
                logger.info(f'Location: {result.label} (p={result.confidence:.2f}, '
                            f'{result.access_points} access points)')
            self._last_label = result.label
        elif result.status != self._last_status or now - self._last_log_time >= REPEAT_LOG_S:
            logger.warn(result.message)
            self._last_log_time = now
        self._last_status = result.status


def main(args=None):
    rclpy.init(args=args)
    node = WhereamiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

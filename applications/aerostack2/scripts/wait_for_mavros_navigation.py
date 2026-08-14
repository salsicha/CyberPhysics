#!/usr/bin/env python3
"""Configure telemetry and gate autonomy until ArduPilot has usable position."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Required by MAVROS' sensor, GPS, local-position, and velocity plugins.
MESSAGE_RATES_HZ = {
    24: 5.0,   # GPS_RAW_INT
    30: 20.0,  # ATTITUDE
    31: 20.0,  # ATTITUDE_QUATERNION
    32: 20.0,  # LOCAL_POSITION_NED
    33: 5.0,   # GLOBAL_POSITION_INT
    74: 5.0,   # VFR_HUD
    147: 2.0,  # BATTERY_STATUS
    245: 2.0,  # EXTENDED_SYS_STATE
}


class NavigationReadiness(Node):
    def __init__(self, namespace):
        super().__init__("navigator_navigation_readiness")
        self.state = None
        self.fix = None
        self.odom = None
        self.intervals_configured = False
        self.create_subscription(State, f"{namespace}/state", self._state, 10)
        self.create_subscription(
            NavSatFix, f"{namespace}/global_position/global", self._fix,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, f"{namespace}/local_position/odom", self._odom,
            qos_profile_sensor_data)
        self.interval_client = self.create_client(
            MessageInterval, f"{namespace}/set_message_interval")

    def _state(self, message):
        self.state = message

    def _fix(self, message):
        self.fix = message

    def _odom(self, message):
        self.odom = message

    def configure_intervals(self):
        if self.intervals_configured:
            return True
        if not (self.state and self.state.connected and
                self.interval_client.service_is_ready()):
            return False
        for message_id, rate_hz in MESSAGE_RATES_HZ.items():
            request = MessageInterval.Request()
            request.message_id = message_id
            request.message_rate = rate_hz
            future = self.interval_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is None or not response.success:
                self.get_logger().warning(
                    f"MAVLink interval request failed for message {message_id}; retrying")
                return False
        self.intervals_configured = True
        self.get_logger().info("Configured required MAVLink telemetry intervals")
        return True

    def ready(self):
        fix_valid = (
            self.fix is not None and
            self.fix.status.status >= NavSatStatus.STATUS_FIX and
            math.isfinite(self.fix.latitude) and math.isfinite(self.fix.longitude))
        odom_valid = self.odom is not None and all(math.isfinite(value) for value in (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z,
        ))
        return bool(self.state and self.state.connected and fix_valid and odom_valid)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavigationReadiness(args.namespace.rstrip("/"))
    deadline = time.monotonic() + args.timeout
    ready_since = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.configure_intervals()
            if node.intervals_configured and node.ready():
                ready_since = ready_since or time.monotonic()
                if time.monotonic() - ready_since >= args.settle_seconds:
                    node.get_logger().info(
                        "Navigator ready: MAVROS connected with GPS and local odometry")
                    return
            else:
                ready_since = None
        raise RuntimeError(
            "Navigator did not produce stable GPS and local odometry before timeout")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

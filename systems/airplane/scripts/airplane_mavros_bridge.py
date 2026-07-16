#!/usr/bin/env python3

from __future__ import annotations

from functools import partial

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix


def normalize_namespace(value: str) -> str:
    cleaned = value.strip("/")
    return f"/{cleaned}" if cleaned else ""


class AirplaneMavrosBridge(Node):
    def __init__(self) -> None:
        super().__init__("airplane_mavros_bridge")
        self.declare_parameter("namespace", "plane0")
        self.declare_parameter("mavros_namespace", "/mavros")

        namespace = normalize_namespace(str(self.get_parameter("namespace").value))
        mavros_namespace = normalize_namespace(
            str(self.get_parameter("mavros_namespace").value)
        )
        output_prefix = f"{namespace}/sensor_measurements"

        self._mirror(
            NavSatFix,
            f"{mavros_namespace}/global_position/global",
            f"{output_prefix}/gps",
        )
        self._mirror(
            Imu,
            f"{mavros_namespace}/imu/data",
            f"{output_prefix}/imu",
        )
        self._mirror(
            BatteryState,
            f"{mavros_namespace}/battery",
            f"{output_prefix}/battery",
        )

        self._mirror(
            Odometry,
            f"{mavros_namespace}/local_position/odom",
            f"{output_prefix}/odom",
        )

        self.get_logger().info(
            f"Airplane MAVROS bridge publishing telemetry under {output_prefix}"
        )

    def _mirror(self, msg_type, input_topic: str, output_topic: str) -> None:
        publisher = self.create_publisher(
            msg_type, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            msg_type,
            input_topic,
            partial(self._publish, publisher),
            qos_profile_sensor_data,
        )

    @staticmethod
    def _publish(publisher, msg) -> None:
        publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = AirplaneMavrosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

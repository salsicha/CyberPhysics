#!/usr/bin/env python3

from __future__ import annotations

from functools import partial

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix


def normalize_namespace(value: str) -> str:
    cleaned = value.strip("/")
    return f"/{cleaned}" if cleaned else ""


class BlueBoatBridge(Node):
    def __init__(self) -> None:
        super().__init__("blueboat_bridge")
        self.declare_parameter("namespace", "boat0")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("enable_velocity_setpoints", False)
        self.declare_parameter("cmd_vel_topic", "/boat0/cmd_vel")
        self.declare_parameter(
            "mavros_velocity_topic",
            "/mavros/setpoint_velocity/cmd_vel",
        )
        self.declare_parameter("velocity_frame_id", "map")

        namespace = normalize_namespace(
            str(self.get_parameter("namespace").value))
        mavros_namespace = normalize_namespace(
            str(self.get_parameter("mavros_namespace").value))
        output_prefix = f"{namespace}/sensor_measurements"

        self._mirror(
            NavSatFix,
            f"{mavros_namespace}/global_position/global",
            f"{output_prefix}/gps",
        )
        self._mirror(
            Odometry,
            f"{mavros_namespace}/local_position/odom",
            f"{output_prefix}/odom",
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

        self.velocity_frame_id = str(
            self.get_parameter("velocity_frame_id").value)
        self.velocity_pub = None
        if bool(self.get_parameter("enable_velocity_setpoints").value):
            self.velocity_pub = self.create_publisher(
                TwistStamped,
                str(self.get_parameter("mavros_velocity_topic").value),
                10,
            )
            self.create_subscription(
                Twist,
                str(self.get_parameter("cmd_vel_topic").value),
                self._cmd_vel,
                10,
            )
            self.get_logger().warn(
                "Boat velocity setpoints are enabled. Verify ArduRover "
                "mode, arming, and manual override before operating.")

        self.get_logger().info(
            f"BlueBoat bridge publishing telemetry under {output_prefix}")

    def _mirror(self, msg_type, input_topic: str, output_topic: str) -> None:
        publisher = self.create_publisher(
            msg_type, output_topic, qos_profile_sensor_data)
        self.create_subscription(
            msg_type,
            input_topic,
            partial(self._publish, publisher),
            qos_profile_sensor_data,
        )

    @staticmethod
    def _publish(publisher, msg) -> None:
        publisher.publish(msg)

    def _cmd_vel(self, msg: Twist) -> None:
        if self.velocity_pub is None:
            return
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.velocity_frame_id
        out.twist = msg
        self.velocity_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = BlueBoatBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class HarborWaypointNav(Node):
    def __init__(self) -> None:
        super().__init__("harbor_waypoint_nav")
        self.declare_parameter("odom_topic", "/boat0/sensor_measurements/odom")
        self.declare_parameter("cmd_vel_topic", "/boat0/cmd_vel")
        self.declare_parameter("waypoints_xy", [0.0, 0.0, 25.0, 0.0])
        self.declare_parameter("loop", False)
        self.declare_parameter("acceptance_radius_m", 1.5)
        self.declare_parameter("desired_speed_mps", 1.1)
        self.declare_parameter("slow_radius_m", 8.0)
        self.declare_parameter("heading_gain", 1.4)
        self.declare_parameter("max_yaw_rate_rps", 0.55)

        flat = [float(v) for v in self.get_parameter("waypoints_xy").value]
        if len(flat) < 4 or len(flat) % 2 != 0:
            raise ValueError("waypoints_xy must contain x/y pairs")
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.index = 0
        self.complete = False
        self.loop = bool(self.get_parameter("loop").value)
        self.acceptance_radius = float(
            self.get_parameter("acceptance_radius_m").value)
        self.desired_speed = float(self.get_parameter("desired_speed_mps").value)
        self.slow_radius = max(0.1, float(self.get_parameter("slow_radius_m").value))
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Harbor survey scenario loaded with {len(self.waypoints)} waypoints")

    def _odom(self, msg: Odometry) -> None:
        if self.complete:
            self.cmd_pub.publish(Twist())
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        target_x, target_y = self.waypoints[self.index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        if distance <= self.acceptance_radius:
            if self.index < len(self.waypoints) - 1:
                self.index += 1
                self.get_logger().info(f"Advancing to waypoint {self.index}")
            elif self.loop:
                self.index = 0
                self.get_logger().info("Restarting harbor survey loop")
            else:
                self.complete = True
                self.get_logger().info("Harbor survey complete")
                self.cmd_pub.publish(Twist())
                return

        target_x, target_y = self.waypoints[self.index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = angle_error(desired_heading, yaw)
        speed = self.desired_speed * min(1.0, distance / self.slow_radius)
        speed *= max(0.15, math.cos(heading_error))

        cmd = Twist()
        cmd.linear.x = clamp(speed, 0.0, self.desired_speed)
        cmd.angular.z = clamp(
            self.heading_gain * heading_error,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = HarborWaypointNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

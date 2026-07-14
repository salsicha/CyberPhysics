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


class PipelineWaypointNav(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_waypoint_nav")
        self.declare_parameter("odom_topic", "/sub0/sensor_measurements/odom")
        self.declare_parameter("cmd_vel_topic", "/sub0/cmd_vel")
        self.declare_parameter(
            "waypoints_x_y_depth",
            [0.0, 0.0, 2.0, 12.0, 0.0, 3.0],
        )
        self.declare_parameter("loop", False)
        self.declare_parameter("acceptance_radius_m", 0.9)
        self.declare_parameter("depth_tolerance_m", 0.35)
        self.declare_parameter("desired_speed_mps", 0.45)
        self.declare_parameter("slow_radius_m", 4.0)
        self.declare_parameter("heading_gain", 1.2)
        self.declare_parameter("depth_gain", 0.45)
        self.declare_parameter("max_yaw_rate_rps", 0.4)
        self.declare_parameter("max_vertical_speed_mps", 0.25)

        flat = [
            float(v) for v in
            self.get_parameter("waypoints_x_y_depth").value
        ]
        if len(flat) < 6 or len(flat) % 3 != 0:
            raise ValueError("waypoints_x_y_depth must contain x/y/depth triples")
        self.waypoints = list(zip(flat[0::3], flat[1::3], flat[2::3]))
        self.index = 0
        self.complete = False
        self.loop = bool(self.get_parameter("loop").value)
        self.acceptance_radius = float(
            self.get_parameter("acceptance_radius_m").value)
        self.depth_tolerance = float(
            self.get_parameter("depth_tolerance_m").value)
        self.desired_speed = float(self.get_parameter("desired_speed_mps").value)
        self.slow_radius = max(0.1, float(self.get_parameter("slow_radius_m").value))
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.depth_gain = float(self.get_parameter("depth_gain").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.max_vertical = float(
            self.get_parameter("max_vertical_speed_mps").value)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Pipeline inspection scenario loaded with {len(self.waypoints)} waypoints")

    def _odom(self, msg: Odometry) -> None:
        if self.complete:
            self.cmd_pub.publish(Twist())
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        depth = -float(msg.pose.pose.position.z)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        target_x, target_y, target_depth = self.waypoints[self.index]
        horizontal_distance = math.hypot(target_x - x, target_y - y)
        depth_error = target_depth - depth
        if (
            horizontal_distance <= self.acceptance_radius and
            abs(depth_error) <= self.depth_tolerance
        ):
            if self.index < len(self.waypoints) - 1:
                self.index += 1
                self.get_logger().info(f"Advancing to waypoint {self.index}")
            elif self.loop:
                self.index = 0
                self.get_logger().info("Restarting pipeline inspection loop")
            else:
                self.complete = True
                self.get_logger().info("Pipeline inspection complete")
                self.cmd_pub.publish(Twist())
                return

        target_x, target_y, target_depth = self.waypoints[self.index]
        dx = target_x - x
        dy = target_y - y
        horizontal_distance = math.hypot(dx, dy)
        depth_error = target_depth - depth
        desired_heading = math.atan2(dy, dx)
        heading_error = angle_error(desired_heading, yaw)
        speed = self.desired_speed * min(1.0, horizontal_distance / self.slow_radius)
        speed *= max(0.1, math.cos(heading_error))

        cmd = Twist()
        cmd.linear.x = clamp(speed, 0.0, self.desired_speed)
        cmd.linear.z = clamp(
            -self.depth_gain * depth_error,
            -self.max_vertical,
            self.max_vertical,
        )
        cmd.angular.z = clamp(
            self.heading_gain * heading_error,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = PipelineWaypointNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

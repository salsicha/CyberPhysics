#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Illuminance
from std_msgs.msg import Float32, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class LunarWaypointNav(Node):
    def __init__(self) -> None:
        super().__init__("lunar_waypoint_nav")
        self.declare_parameter("odom_topic", "/lunar_rover/odom")
        self.declare_parameter("cmd_vel_topic", "/lunar_rover/cmd_vel")
        self.declare_parameter("slope_topic", "/lunar_rover/terrain/slope_deg")
        self.declare_parameter("slip_topic", "/lunar_rover/terrain/slip_ratio")
        self.declare_parameter("illumination_topic", "/lunar_rover/solar/illumination")
        self.declare_parameter("battery_topic", "/lunar_rover/battery")
        self.declare_parameter("mission_status_topic", "/lunar_rover/mission_status")
        self.declare_parameter("waypoints_xy", [0.0, -4.0, 12.0, -4.0])
        self.declare_parameter("loop", False)
        self.declare_parameter("acceptance_radius_m", 1.0)
        self.declare_parameter("desired_speed_mps", 0.28)
        self.declare_parameter("slow_radius_m", 5.5)
        self.declare_parameter("heading_gain", 1.1)
        self.declare_parameter("max_yaw_rate_rps", 0.34)
        self.declare_parameter("shadow_slowdown_factor", 0.45)
        self.declare_parameter("low_illumination_lux", 25000.0)
        self.declare_parameter("slip_slowdown_start", 0.18)
        self.declare_parameter("slip_stop_threshold", 0.52)
        self.declare_parameter("slope_slowdown_deg", 12.0)
        self.declare_parameter("battery_reserve_fraction", 0.12)

        flat = [float(v) for v in self.get_parameter("waypoints_xy").value]
        if len(flat) < 4 or len(flat) % 2 != 0:
            raise ValueError("waypoints_xy must contain x/y pairs")
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.index = 0
        self.loop = bool(self.get_parameter("loop").value)
        self.acceptance_radius = float(
            self.get_parameter("acceptance_radius_m").value
        )
        self.desired_speed = float(self.get_parameter("desired_speed_mps").value)
        self.slow_radius = max(0.1, float(self.get_parameter("slow_radius_m").value))
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.shadow_slowdown = float(self.get_parameter("shadow_slowdown_factor").value)
        self.low_illumination = float(self.get_parameter("low_illumination_lux").value)
        self.slip_slowdown_start = float(self.get_parameter("slip_slowdown_start").value)
        self.slip_stop_threshold = float(self.get_parameter("slip_stop_threshold").value)
        self.slope_slowdown = float(self.get_parameter("slope_slowdown_deg").value)
        self.battery_reserve = float(self.get_parameter("battery_reserve_fraction").value)

        self.slope_deg = 0.0
        self.slip_ratio = 0.0
        self.illumination_lux = 110000.0
        self.battery_fraction = 1.0
        self.complete = False

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("mission_status_topic").value), 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("slope_topic").value), self._slope, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("slip_topic").value), self._slip, 10
        )
        self.create_subscription(
            Illuminance,
            str(self.get_parameter("illumination_topic").value),
            self._illumination,
            10,
        )
        self.create_subscription(
            BatteryState,
            str(self.get_parameter("battery_topic").value),
            self._battery,
            10,
        )
        self.get_logger().info(
            f"South-pole prospecting route loaded with {len(self.waypoints)} waypoints"
        )

    def _slope(self, msg: Float32) -> None:
        self.slope_deg = float(msg.data)

    def _slip(self, msg: Float32) -> None:
        self.slip_ratio = float(msg.data)

    def _illumination(self, msg: Illuminance) -> None:
        self.illumination_lux = float(msg.illuminance)

    def _battery(self, msg: BatteryState) -> None:
        self.battery_fraction = float(msg.percentage)

    def _odom(self, msg: Odometry) -> None:
        if self.complete:
            self.cmd_pub.publish(Twist())
            return

        if self.battery_fraction <= self.battery_reserve:
            self._publish_status("hold: battery reserve reached")
            self.cmd_pub.publish(Twist())
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        target_x, target_y = self.waypoints[self.index]
        distance = math.hypot(target_x - x, target_y - y)
        if distance <= self.acceptance_radius:
            if self.index < len(self.waypoints) - 1:
                self.index += 1
                self.get_logger().info(f"Advancing to waypoint {self.index}")
            elif self.loop:
                self.index = 0
                self.get_logger().info("Restarting lunar traverse loop")
            else:
                self.complete = True
                self._publish_status("complete: south-pole traverse finished")
                self.cmd_pub.publish(Twist())
                return

        target_x, target_y = self.waypoints[self.index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        heading_error = angle_error(math.atan2(dy, dx), yaw)
        speed = self.desired_speed * min(1.0, distance / self.slow_radius)
        speed *= max(0.18, math.cos(heading_error))
        speed *= self._terrain_speed_factor()

        cmd = Twist()
        cmd.linear.x = clamp(speed, 0.0, self.desired_speed)
        cmd.angular.z = clamp(
            self.heading_gain * heading_error,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self.cmd_pub.publish(cmd)
        self._publish_status(
            "enroute: waypoint="
            f"{self.index} distance_m={distance:.1f} slip={self.slip_ratio:.2f} "
            f"slope_deg={self.slope_deg:.1f} illumination_lux={self.illumination_lux:.0f}"
        )

    def _terrain_speed_factor(self) -> float:
        factor = 1.0
        if self.illumination_lux < self.low_illumination:
            factor *= self.shadow_slowdown
        if self.slope_deg > self.slope_slowdown:
            slope_excess = self.slope_deg - self.slope_slowdown
            factor *= clamp(1.0 - 0.04 * slope_excess, 0.35, 1.0)
        if self.slip_ratio >= self.slip_stop_threshold:
            factor *= 0.2
        elif self.slip_ratio > self.slip_slowdown_start:
            span = max(0.01, self.slip_stop_threshold - self.slip_slowdown_start)
            slip_fraction = (self.slip_ratio - self.slip_slowdown_start) / span
            factor *= clamp(1.0 - 0.55 * slip_fraction, 0.35, 1.0)
        return clamp(factor, 0.12, 1.0)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LunarWaypointNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

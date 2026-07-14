#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    msg = Quaternion()
    msg.z = math.sin(yaw * 0.5)
    msg.w = math.cos(yaw * 0.5)
    return msg


class BlueBoatSim(Node):
    def __init__(self) -> None:
        super().__init__("blueboat_sim")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("origin_lat", 37.79552)
        self.declare_parameter("origin_lon", -122.28134)
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("current_x_mps", 0.08)
        self.declare_parameter("current_y_mps", -0.03)
        self.declare_parameter("wave_sway_mps", 0.04)
        self.declare_parameter("max_speed_mps", 1.8)
        self.declare_parameter("max_yaw_rate_rps", 0.65)
        self.declare_parameter("speed_time_constant_s", 1.4)
        self.declare_parameter("yaw_time_constant_s", 0.8)
        self.declare_parameter("battery_runtime_s", 3600.0)
        self.declare_parameter("cmd_timeout_s", 5.0)
        self.declare_parameter(
            "mavros_velocity_topic", "/mavros/setpoint_velocity/cmd_vel")

        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        self.x = float(self.get_parameter("initial_x_m").value)
        self.y = float(self.get_parameter("initial_y_m").value)
        self.yaw = float(self.get_parameter("initial_yaw_rad").value)
        self.current_x = float(self.get_parameter("current_x_mps").value)
        self.current_y = float(self.get_parameter("current_y_mps").value)
        self.wave_sway = float(self.get_parameter("wave_sway_mps").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_yaw_rate = float(
            self.get_parameter("max_yaw_rate_rps").value)
        self.speed_tau = max(
            0.05, float(self.get_parameter("speed_time_constant_s").value))
        self.yaw_tau = max(
            0.05, float(self.get_parameter("yaw_time_constant_s").value))
        self.battery_runtime = max(
            1.0, float(self.get_parameter("battery_runtime_s").value))
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.cmd_speed = 0.0
        self.cmd_yaw_rate = 0.0
        self.elapsed = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry, "/mavros/local_position/odom", 10)
        self.gps_pub = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", 10)
        self.imu_pub = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self.battery_pub = self.create_publisher(
            BatteryState, "/mavros/battery", 10)
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("mavros_velocity_topic").value),
            self._command,
            10,
        )
        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            "BlueBoat simulator running harbor survey dynamics")

    def _command(self, msg: TwistStamped) -> None:
        self.cmd_speed = clamp(
            float(msg.twist.linear.x), -self.max_speed, self.max_speed)
        self.cmd_yaw_rate = clamp(
            float(msg.twist.angular.z), -self.max_yaw_rate, self.max_yaw_rate)
        self.last_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        self.elapsed += dt
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd_speed = 0.0
            self.cmd_yaw_rate = 0.0
        self.speed += (self.cmd_speed - self.speed) * min(1.0, dt / self.speed_tau)
        self.yaw_rate += (
            self.cmd_yaw_rate - self.yaw_rate) * min(1.0, dt / self.yaw_tau)
        self.yaw = math.atan2(
            math.sin(self.yaw + self.yaw_rate * dt),
            math.cos(self.yaw + self.yaw_rate * dt),
        )
        wave = self.wave_sway * math.sin(0.35 * self.elapsed)
        self.x += (
            self.speed * math.cos(self.yaw) + self.current_x -
            wave * math.sin(self.yaw)) * dt
        self.y += (
            self.speed * math.sin(self.yaw) + self.current_y +
            wave * math.cos(self.yaw)) * dt

        stamp = now.to_msg()
        quat = yaw_to_quaternion(self.yaw)
        self._publish_odom(stamp, quat)
        self._publish_gps(stamp)
        self._publish_imu(stamp, quat)
        self._publish_battery(stamp)

    def _publish_odom(self, stamp, quat: Quaternion) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "boat0/base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quat
        msg.twist.twist.linear.x = self.speed
        msg.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(msg)

    def _publish_gps(self, stamp) -> None:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = "boat0/gps_link"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        metres_per_deg_lat = 111_111.0
        metres_per_deg_lon = metres_per_deg_lat * math.cos(
            math.radians(self.origin_lat))
        msg.latitude = self.origin_lat + self.y / metres_per_deg_lat
        msg.longitude = self.origin_lon + self.x / metres_per_deg_lon
        msg.altitude = 0.0
        self.gps_pub.publish(msg)

    def _publish_imu(self, stamp, quat: Quaternion) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "boat0/imu_link"
        msg.orientation = quat
        msg.angular_velocity.z = self.yaw_rate
        self.imu_pub.publish(msg)

    def _publish_battery(self, stamp) -> None:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = "boat0/battery"
        msg.voltage = 15.6
        msg.current = -8.0 - 4.0 * abs(self.speed)
        msg.percentage = clamp(1.0 - self.elapsed / self.battery_runtime, 0.0, 1.0)
        self.battery_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BlueBoatSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

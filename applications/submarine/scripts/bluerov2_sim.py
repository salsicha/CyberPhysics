#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, NavSatFix, NavSatStatus
from synthetic_world import local_to_latlon


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    msg = Quaternion()
    msg.z = math.sin(yaw * 0.5)
    msg.w = math.cos(yaw * 0.5)
    return msg


class BlueROV2Sim(Node):
    def __init__(self) -> None:
        super().__init__("bluerov2_sim")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("origin_lat", 37.80750)
        self.declare_parameter("origin_lon", -122.41700)
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_depth_m", 1.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("current_x_mps", 0.04)
        self.declare_parameter("current_y_mps", 0.02)
        self.declare_parameter("max_forward_speed_mps", 0.75)
        self.declare_parameter("max_vertical_speed_mps", 0.35)
        self.declare_parameter("max_yaw_rate_rps", 0.45)
        self.declare_parameter("velocity_time_constant_s", 1.2)
        self.declare_parameter("battery_runtime_s", 2700.0)
        self.declare_parameter("cmd_timeout_s", 5.0)
        self.declare_parameter(
            "mavros_velocity_topic", "/mavros/setpoint_velocity/cmd_vel")

        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        self.x = float(self.get_parameter("initial_x_m").value)
        self.y = float(self.get_parameter("initial_y_m").value)
        self.depth = max(0.0, float(self.get_parameter("initial_depth_m").value))
        self.yaw = float(self.get_parameter("initial_yaw_rad").value)
        self.current_x = float(self.get_parameter("current_x_mps").value)
        self.current_y = float(self.get_parameter("current_y_mps").value)
        self.max_forward = float(
            self.get_parameter("max_forward_speed_mps").value)
        self.max_vertical = float(
            self.get_parameter("max_vertical_speed_mps").value)
        self.max_yaw_rate = float(
            self.get_parameter("max_yaw_rate_rps").value)
        self.velocity_tau = max(
            0.05, float(self.get_parameter("velocity_time_constant_s").value))
        self.battery_runtime = max(
            1.0, float(self.get_parameter("battery_runtime_s").value))
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self.forward_speed = 0.0
        self.vertical_speed = 0.0
        self.yaw_rate = 0.0
        self.cmd_forward = 0.0
        self.cmd_vertical = 0.0
        self.cmd_yaw_rate = 0.0
        self.elapsed = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry, "/mavros/local_position/odom", 10)
        self.gps_pub = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", 10)
        self.imu_pub = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self.pressure_pub = self.create_publisher(
            FluidPressure, "/mavros/imu/static_pressure", 10)
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
            "BlueROV2 simulator running submerged inspection dynamics")

    def _command(self, msg: TwistStamped) -> None:
        self.cmd_forward = clamp(
            float(msg.twist.linear.x), -self.max_forward, self.max_forward)
        self.cmd_vertical = clamp(
            float(msg.twist.linear.z), -self.max_vertical, self.max_vertical)
        self.cmd_yaw_rate = clamp(
            float(msg.twist.angular.z), -self.max_yaw_rate, self.max_yaw_rate)
        self.last_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        self.elapsed += dt
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd_forward = 0.0
            self.cmd_vertical = 0.0
            self.cmd_yaw_rate = 0.0
        alpha = min(1.0, dt / self.velocity_tau)
        self.forward_speed += (self.cmd_forward - self.forward_speed) * alpha
        self.vertical_speed += (self.cmd_vertical - self.vertical_speed) * alpha
        self.yaw_rate += (self.cmd_yaw_rate - self.yaw_rate) * alpha
        self.yaw = math.atan2(
            math.sin(self.yaw + self.yaw_rate * dt),
            math.cos(self.yaw + self.yaw_rate * dt),
        )

        self.x += (self.forward_speed * math.cos(self.yaw) + self.current_x) * dt
        self.y += (self.forward_speed * math.sin(self.yaw) + self.current_y) * dt
        self.depth = max(0.0, self.depth - self.vertical_speed * dt)

        stamp = now.to_msg()
        quat = yaw_to_quaternion(self.yaw)
        self._publish_odom(stamp, quat)
        self._publish_gps(stamp)
        self._publish_imu(stamp, quat)
        self._publish_pressure(stamp)
        self._publish_battery(stamp)

    def _publish_odom(self, stamp, quat: Quaternion) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "sub0/base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = -self.depth
        msg.pose.pose.orientation = quat
        msg.twist.twist.linear.x = self.forward_speed
        msg.twist.twist.linear.z = self.vertical_speed
        msg.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(msg)

    def _publish_gps(self, stamp) -> None:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = "sub0/gps_link"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude = local_to_latlon(
            self.x, self.y, self.origin_lat, self.origin_lon)
        msg.altitude = -self.depth
        self.gps_pub.publish(msg)

    def _publish_imu(self, stamp, quat: Quaternion) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "sub0/imu_link"
        msg.orientation = quat
        msg.angular_velocity.z = self.yaw_rate
        self.imu_pub.publish(msg)

    def _publish_pressure(self, stamp) -> None:
        msg = FluidPressure()
        msg.header.stamp = stamp
        msg.header.frame_id = "sub0/pressure_link"
        msg.fluid_pressure = 101_325.0 + 1025.0 * 9.80665 * self.depth
        msg.variance = 25.0
        self.pressure_pub.publish(msg)

    def _publish_battery(self, stamp) -> None:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = "sub0/battery"
        msg.voltage = 15.4
        msg.current = -6.0 - 5.0 * abs(self.forward_speed)
        msg.percentage = clamp(1.0 - self.elapsed / self.battery_runtime, 0.0, 1.0)
        self.battery_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BlueROV2Sim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

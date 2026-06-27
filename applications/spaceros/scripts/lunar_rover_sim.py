#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Illuminance, Imu, JointState, Temperature
from std_msgs.msg import Float32


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    msg = Quaternion()
    msg.z = math.sin(yaw * 0.5)
    msg.w = math.cos(yaw * 0.5)
    return msg


class LunarRoverSim(Node):
    def __init__(self) -> None:
        super().__init__("lunar_rover_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_topic", "/lunar_rover/odom")
        self.declare_parameter("cmd_vel_topic", "/lunar_rover/cmd_vel")
        self.declare_parameter("imu_topic", "/lunar_rover/imu")
        self.declare_parameter("battery_topic", "/lunar_rover/battery")
        self.declare_parameter("joint_state_topic", "/lunar_rover/wheel_states")
        self.declare_parameter("slope_topic", "/lunar_rover/terrain/slope_deg")
        self.declare_parameter("slip_topic", "/lunar_rover/terrain/slip_ratio")
        self.declare_parameter("illumination_topic", "/lunar_rover/solar/illumination")
        self.declare_parameter("temperature_topic", "/lunar_rover/thermal/body")
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", -4.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("max_speed_mps", 0.38)
        self.declare_parameter("max_yaw_rate_rps", 0.42)
        self.declare_parameter("speed_time_constant_s", 2.0)
        self.declare_parameter("yaw_time_constant_s", 1.4)
        self.declare_parameter("wheel_radius_m", 0.18)
        self.declare_parameter("battery_capacity_wh", 650.0)
        self.declare_parameter("initial_battery_wh", 585.0)
        self.declare_parameter("idle_power_w", 42.0)
        self.declare_parameter("drive_power_w", 115.0)
        self.declare_parameter("solar_charge_w", 38.0)

        self.x = float(self.get_parameter("initial_x_m").value)
        self.y = float(self.get_parameter("initial_y_m").value)
        self.yaw = float(self.get_parameter("initial_yaw_rad").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.speed_tau = max(
            0.05, float(self.get_parameter("speed_time_constant_s").value)
        )
        self.yaw_tau = max(
            0.05, float(self.get_parameter("yaw_time_constant_s").value)
        )
        self.wheel_radius = max(0.01, float(self.get_parameter("wheel_radius_m").value))
        self.battery_capacity_wh = max(
            1.0, float(self.get_parameter("battery_capacity_wh").value)
        )
        self.energy_wh = clamp(
            float(self.get_parameter("initial_battery_wh").value),
            0.0,
            self.battery_capacity_wh,
        )
        self.idle_power_w = float(self.get_parameter("idle_power_w").value)
        self.drive_power_w = float(self.get_parameter("drive_power_w").value)
        self.solar_charge_w = float(self.get_parameter("solar_charge_w").value)

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.cmd_speed = 0.0
        self.cmd_yaw_rate = 0.0
        self.elapsed = 0.0
        self.slip_ratio = 0.04
        self.wheel_angle = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), 10
        )
        self.battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter("battery_topic").value), 10
        )
        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.slope_pub = self.create_publisher(
            Float32, str(self.get_parameter("slope_topic").value), 10
        )
        self.slip_pub = self.create_publisher(
            Float32, str(self.get_parameter("slip_topic").value), 10
        )
        self.illumination_pub = self.create_publisher(
            Illuminance, str(self.get_parameter("illumination_topic").value), 10
        )
        self.temperature_pub = self.create_publisher(
            Temperature, str(self.get_parameter("temperature_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._command,
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("Lunar rover simulator running south-pole terrain")

    def _command(self, msg: Twist) -> None:
        self.cmd_speed = clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        self.cmd_yaw_rate = clamp(
            float(msg.angular.z), -self.max_yaw_rate, self.max_yaw_rate
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        self.elapsed += dt

        slope_deg, slip_base, illumination_lux, temperature_c = self._terrain_metrics()
        command_load = abs(self.cmd_speed) / max(0.01, self.max_speed)
        self.slip_ratio = clamp(
            slip_base + 0.004 * slope_deg + 0.08 * command_load, 0.02, 0.65
        )
        traction = 1.0 - self.slip_ratio
        target_speed = self.cmd_speed * traction
        yaw_traction = clamp(1.0 - 0.55 * self.slip_ratio, 0.35, 1.0)
        target_yaw_rate = self.cmd_yaw_rate * yaw_traction

        self.speed += (target_speed - self.speed) * min(1.0, dt / self.speed_tau)
        self.yaw_rate += (
            target_yaw_rate - self.yaw_rate
        ) * min(1.0, dt / self.yaw_tau)
        self.yaw = math.atan2(
            math.sin(self.yaw + self.yaw_rate * dt),
            math.cos(self.yaw + self.yaw_rate * dt),
        )
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        self.wheel_angle += (self.speed / self.wheel_radius) * dt

        charge_factor = clamp(illumination_lux / 110000.0, 0.0, 1.0)
        drain_w = self.idle_power_w + self.drive_power_w * abs(self.speed) / self.max_speed
        charge_w = self.solar_charge_w * charge_factor
        self.energy_wh = clamp(
            self.energy_wh + (charge_w - drain_w) * dt / 3600.0,
            0.0,
            self.battery_capacity_wh,
        )

        stamp = now.to_msg()
        quat = yaw_to_quaternion(self.yaw)
        self._publish_odom(stamp, quat)
        self._publish_imu(stamp, quat, slope_deg)
        self._publish_battery(stamp)
        self._publish_wheels(stamp)
        self._publish_environment(stamp, slope_deg, illumination_lux, temperature_c)

    def _terrain_metrics(self) -> tuple[float, float, float, float]:
        crater_dx = self.x - 50.0
        crater_dy = self.y - 14.0
        crater_radius = math.hypot(crater_dx, crater_dy)
        rim = math.exp(-((crater_radius - 15.0) / 4.0) ** 2)
        ridge = math.exp(-((self.y - (0.28 * self.x - 3.0)) / 5.0) ** 2) * math.exp(
            -((self.x - 34.0) / 27.0) ** 2
        )
        loose_regolith = math.exp(
            -(((self.x - 24.0) / 8.0) ** 2 + ((self.y + 8.0) / 5.0) ** 2)
        )
        shadow = 1.0 if crater_radius < 12.5 and self.y > 8.0 else 0.0
        moving_penumbra = 0.5 + 0.5 * math.sin(0.045 * self.x - 0.03 * self.y)
        penumbra = clamp(rim * moving_penumbra, 0.0, 0.65)
        illumination = 110000.0 * clamp(1.0 - 0.90 * shadow - 0.55 * penumbra, 0.035, 1.0)
        slope_deg = 2.0 + 9.5 * rim + 5.0 * ridge + 4.5 * loose_regolith
        slip_base = 0.035 + 0.12 * loose_regolith + 0.075 * ridge + 0.08 * rim
        temperature_c = -135.0 + 185.0 * clamp(illumination / 110000.0, 0.0, 1.0)
        return slope_deg, slip_base, illumination, temperature_c

    def _publish_odom(self, stamp, quat: Quaternion) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "lunar_map"
        msg.child_frame_id = "lunar_rover/base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quat
        msg.twist.twist.linear.x = self.speed
        msg.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(msg)

    def _publish_imu(self, stamp, quat: Quaternion, slope_deg: float) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "lunar_rover/imu_link"
        msg.orientation = quat
        msg.angular_velocity.z = self.yaw_rate
        msg.linear_acceleration.x = math.sin(math.radians(slope_deg)) * 1.62
        msg.linear_acceleration.z = math.cos(math.radians(slope_deg)) * 1.62
        self.imu_pub.publish(msg)

    def _publish_battery(self, stamp) -> None:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = "lunar_rover/battery"
        msg.voltage = 28.0 * clamp(self.energy_wh / self.battery_capacity_wh, 0.72, 1.0)
        msg.current = -(
            self.idle_power_w + self.drive_power_w * abs(self.speed) / self.max_speed
        ) / max(1.0, msg.voltage)
        msg.percentage = clamp(self.energy_wh / self.battery_capacity_wh, 0.0, 1.0)
        self.battery_pub.publish(msg)

    def _publish_wheels(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "front_left_wheel",
            "front_right_wheel",
            "middle_left_wheel",
            "middle_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
        ]
        msg.position = [self.wheel_angle] * len(msg.name)
        msg.velocity = [self.speed / self.wheel_radius] * len(msg.name)
        self.joint_pub.publish(msg)

    def _publish_environment(
        self, stamp, slope_deg: float, illumination_lux: float, temperature_c: float
    ) -> None:
        slope = Float32()
        slope.data = slope_deg
        self.slope_pub.publish(slope)

        slip = Float32()
        slip.data = self.slip_ratio
        self.slip_pub.publish(slip)

        illumination = Illuminance()
        illumination.header.stamp = stamp
        illumination.header.frame_id = "lunar_rover/solar_panel"
        illumination.illuminance = illumination_lux
        illumination.variance = 250.0
        self.illumination_pub.publish(illumination)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.header.frame_id = "lunar_rover/body"
        temperature.temperature = temperature_c
        temperature.variance = 2.0
        self.temperature_pub.publish(temperature)


def main() -> None:
    rclpy.init()
    node = LunarRoverSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

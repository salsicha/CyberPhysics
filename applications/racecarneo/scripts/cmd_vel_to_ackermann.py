#!/usr/bin/env python3
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('racecarneo_cmd_vel_to_ackermann')
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_speed', 0.35)
        self.declare_parameter('max_steering_angle', 0.42)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('ackermann_topic', 'ackermann_cmd')
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('ackermann_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self._cmd_vel, 10)

    def _cmd_vel(self, msg: Twist):
        speed = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
        if abs(speed) < 1e-6:
            steering = 0.0
        else:
            steering = math.atan2(float(msg.angular.z) * self.wheelbase, speed)
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = speed / self.max_speed if self.max_speed > 0.0 else 0.0
        out.drive.steering_angle = steering / self.max_steering_angle if self.max_steering_angle > 0.0 else 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

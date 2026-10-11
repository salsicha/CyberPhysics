#!/usr/bin/env python3
"""Twist to ackermann_msgs, with manual driving and a latched stop.

/cmd_vel (Nav2) is converted as before. Twists on the teleop topic (the
web UI's joystick) take priority while they keep arriving: /cmd_vel is
ignored until none has arrived for teleop_timeout_s, and when the teleop
stream stops the car is commanded to zero speed (the operator let go or the
UI lost its link). A true on the stop-request topic (transient-local, so a
latched stop survives a restart of this node) holds zero speed with the
steering where it was, whatever either input says, until a false releases
it. The control status (JSON) reports which input drives the car.
"""
import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class CmdVelToAckermann(Node):
    # Monotonic receive times of the last Twist on each input (None: none yet
    # or timed out), the latched stop, and the last command published, which
    # the status reports and the stop keeps the steering of.
    nav_time = None
    teleop_time = None
    stopped = False
    speed = 0.0
    steering = 0.0

    def __init__(self):
        super().__init__('racecarneo_cmd_vel_to_ackermann')
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_speed', 0.35)
        self.declare_parameter('max_steering_angle', 0.42)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('ackermann_topic', 'ackermann_cmd')
        self.declare_parameter('teleop_topic', 'cmd_vel_teleop')
        self.declare_parameter('teleop_timeout_s', 0.5)
        self.declare_parameter('stop_request_topic', 'racecarneo/stop_request')
        self.declare_parameter('control_status_topic', 'racecarneo/control_status')
        self.declare_parameter('status_rate_hz', 2.0)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.teleop_timeout_s = float(self.get_parameter('teleop_timeout_s').value)
        self.pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('ackermann_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('control_status_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self._cmd_vel, 10)
        self.create_subscription(Twist, self.get_parameter('teleop_topic').value, self._cmd_vel_teleop, 10)
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        # The latched subscription receives the last request from the UI's
        # transient-local publisher; the volatile one also matches default-QoS
        # publishers (a plain `ros2 topic pub`), which a transient-local reader cannot.
        for qos in (latched_qos, 10):
            self.create_subscription(Bool, self.get_parameter('stop_request_topic').value, self._stop_request, qos)
        self.create_timer(0.1, self._check_teleop_timeout)
        self.create_timer(1.0 / float(self.get_parameter('status_rate_hz').value), self._publish_status)

    def _teleop_active(self, now):
        return self.teleop_time is not None and now - self.teleop_time <= self.teleop_timeout_s

    def _cmd_vel(self, msg: Twist):
        now = time.monotonic()
        self.nav_time = now
        if self._teleop_active(now):
            return
        self._drive(msg)

    def _cmd_vel_teleop(self, msg: Twist):
        self.teleop_time = time.monotonic()
        self._drive(msg)

    def _drive(self, msg: Twist):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        # min/max would turn NaN into a full-scale command; publish nothing
        # so the downstream command watchdog stops the car.
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.get_logger().warn('Ignoring non-finite cmd_vel', throttle_duration_sec=1.0)
            return
        if self.stopped:
            self._publish(0.0, self.steering)
            return
        # Curvature comes from the requested twist so that limiting the
        # speed slows the car along the same path instead of tightening it.
        if abs(linear) < 1e-6:
            steering = 0.0
        else:
            steering = math.atan(angular * self.wheelbase / linear)
        speed = max(-self.max_speed, min(self.max_speed, linear))
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
        self._publish(speed, steering)

    def _publish(self, speed, steering):
        # ackermann_msgs units: m/s and rad (positive left).
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = speed
        out.drive.steering_angle = steering
        self.pub.publish(out)
        self.speed, self.steering = speed, steering

    def _stop_request(self, msg: Bool):
        stop = bool(msg.data)
        if stop and not self.stopped:
            self.get_logger().warn('Stop requested: holding zero speed')
            self._publish(0.0, self.steering)
        elif self.stopped and not stop:
            self.get_logger().info('Stop released')
        self.stopped = stop

    def _check_teleop_timeout(self):
        if self.teleop_time is not None and not self._teleop_active(time.monotonic()):
            # Deadman: the joystick was released or the UI lost its link.
            self.teleop_time = None
            self._publish(0.0, self.steering)

    def status(self):
        now = time.monotonic()
        if self._teleop_active(now):
            source = 'teleop'
        elif self.nav_time is not None and now - self.nav_time <= self.teleop_timeout_s:
            source = 'nav2'
        else:
            source = 'none'
        return {'source': source, 'stopped': self.stopped,
                'speed_mps': self.speed, 'steering_rad': self.steering}

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status())))


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

#!/usr/bin/env python3
"""Joint-limit safety filter for the SO-101 simulation interface."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from so101_common import JOINT_LIMITS, JOINT_NAMES


class SO101CBFController(Node):
    def __init__(self):
        super().__init__("so101_cbf_controller")
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("safety_margin", 0.02)
        self.declare_parameter("goal", [0.0] * len(JOINT_NAMES))

        self.kp = float(self.get_parameter("kp").value)
        self.rate = max(1e-3, float(self.get_parameter("control_rate").value))
        self.margin = float(self.get_parameter("safety_margin").value)
        self.q_des = np.asarray(self.get_parameter("goal").value, dtype=float)
        if self.q_des.shape != (len(JOINT_NAMES),):
            raise ValueError("goal must contain six joint positions")

        self.q = np.zeros(len(JOINT_NAMES))
        self.state_received = False
        self.command_pub = self.create_publisher(
            Float64MultiArray, "/so101/joint_commands", 10
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )
        self.timer = self.create_timer(1.0 / self.rate, self._control_step)
        self.get_logger().info("SO-101 joint-limit controller initialized")

    def _joint_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        self.q = np.array([positions[name] for name in JOINT_NAMES])
        self.state_received = True

    def _control_step(self):
        if not self.state_received:
            return

        velocity = -self.kp * (self.q - self.q_des)
        target = self.q + velocity / self.rate
        # Keep the next command inside a shrunken safe set. The gripper margin
        # is scaled because its range is linear and much smaller than a radian.
        margins = np.full(len(JOINT_NAMES), self.margin)
        margins[-1] = min(self.margin, 0.002)
        lower = JOINT_LIMITS[:, 0] + margins
        upper = JOINT_LIMITS[:, 1] - margins
        target = np.clip(target, lower, upper)

        message = Float64MultiArray()
        message.data = target.tolist()
        self.command_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SO101CBFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

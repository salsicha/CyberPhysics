#!/usr/bin/env python3
"""Explicit arming and command safety gate for optional SO-101 HIL runs."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from so101_common import JOINT_NAMES, safe_hardware_target


class SO101HILGuard(Node):
    def __init__(self):
        super().__init__("so101_hil_guard")
        self.declare_parameter("armed", False)
        self.declare_parameter("input_command_topic", "/so101/hil/policy_commands")
        self.declare_parameter("hardware_state_topic", "/so101/hardware/joint_states")
        self.declare_parameter("hardware_command_topic", "/so101/hardware/joint_commands")
        self.declare_parameter("state_timeout_s", 0.5)
        self.declare_parameter("max_joint_step", 0.04)

        self.armed = bool(self.get_parameter("armed").value)
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.latest_positions = None
        self.last_state_ns = None
        self.warned_unarmed = False
        self.warned_stale = False

        self.publisher = self.create_publisher(
            Float64MultiArray, self.get_parameter("hardware_command_topic").value, 10
        )
        self.create_subscription(
            JointState,
            self.get_parameter("hardware_state_topic").value,
            self._state,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("input_command_topic").value,
            self._command,
            10,
        )
        state = "ARMED" if self.armed else "DISARMED"
        self.get_logger().warning(f"SO-101 HIL safety gate is {state}")

    def _state(self, msg):
        by_name = {name: index for index, name in enumerate(msg.name)}
        if not all(name in by_name for name in JOINT_NAMES):
            self.get_logger().warning("Ignoring hardware state without all six SO-101 joints")
            return
        self.latest_positions = np.asarray(
            [msg.position[by_name[name]] for name in JOINT_NAMES], dtype=np.float64
        )
        self.last_state_ns = self.get_clock().now().nanoseconds
        self.warned_stale = False

    def _command(self, msg):
        if not self.armed:
            if not self.warned_unarmed:
                self.get_logger().warning(
                    "Policy commands are blocked; restart with SO101_HIL_ARMED=true to enable hardware output"
                )
                self.warned_unarmed = True
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.latest_positions is None or self.last_state_ns is None or (
            now_ns - self.last_state_ns
        ) * 1e-9 > self.state_timeout_s:
            if not self.warned_stale:
                self.get_logger().error("Policy command blocked: hardware joint state is missing or stale")
                self.warned_stale = True
            return
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().error("Policy command blocked: expected six SO-101 joints")
            return
        try:
            target = safe_hardware_target(msg.data, self.latest_positions, self.max_joint_step)
        except ValueError as exc:
            self.get_logger().error(f"Policy command blocked: {exc}")
            return
        output = Float64MultiArray()
        output.data = target.astype(float).tolist()
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = SO101HILGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

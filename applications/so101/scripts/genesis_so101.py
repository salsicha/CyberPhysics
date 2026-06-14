#!/usr/bin/env python3
"""Run the SO-101 in Genesis with the common ROS 2 command interface."""

import argparse
import os
import time

import genesis as gs
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from so101_common import JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS


class GenesisBridge(Node):
    def __init__(self):
        super().__init__("so101_genesis_bridge")
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, "/so101/joint_commands", self._command, 10
        )

    def _command(self, msg):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warning("Expected six SO-101 joint commands")
            return
        self.target = np.clip(
            np.asarray(msg.data), LOWER_LIMITS, UPPER_LIMITS
        ).astype(np.float32)

    def publish_state(self, positions, velocities):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = np.asarray(positions).tolist()
        msg.velocity = np.asarray(velocities).tolist()
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default="/workspace/so101/urdf/so101.urdf")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--backend", choices=("gpu", "cpu"), default="gpu")
    args = parser.parse_args()

    if not os.path.isfile(args.urdf):
        raise FileNotFoundError(args.urdf)

    rclpy.init()
    bridge = GenesisBridge()
    gs.init(backend=getattr(gs, args.backend))
    scene = gs.Scene(show_viewer=not args.headless)
    scene.add_entity(gs.morphs.Plane())
    robot = scene.add_entity(gs.morphs.URDF(file=args.urdf, fixed=True))
    scene.build()

    dofs = [robot.get_joint(name).dof_idx_local for name in JOINT_NAMES]
    robot.set_dofs_kp(np.array([80, 80, 70, 45, 30, 20]), dofs_idx_local=dofs)
    robot.set_dofs_kv(np.array([8, 8, 7, 5, 4, 2]), dofs_idx_local=dofs)

    try:
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.0)
            robot.control_dofs_position(bridge.target, dofs_idx_local=dofs)
            scene.step()
            bridge.publish_state(
                robot.get_dofs_position(dofs_idx_local=dofs),
                robot.get_dofs_velocity(dofs_idx_local=dofs),
            )
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

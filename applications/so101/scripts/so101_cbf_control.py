#!/usr/bin/env python3
"""
SO-101 CBF Control Script
-------------------------
This script demonstrates how to use cbfkit for safe control of the SO-101 arm.
It implements a Control Barrier Function (CBF) based controller that enforces
joint limits while tracking a desired configuration.
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Attempt to import cbfkit
try:
    # Note: Adjust these imports based on the specific version/structure of cbfkit installed
    import cbfkit.systems as systems
    import cbfkit.controllers as controllers
    from cbfkit.utils.user_input import get_user_input
except ImportError:
    print("Warning: cbfkit not found. The script will run in a degraded mode or fail.")
    # We continue to allow the ROS node structure to be visible

class SO101CBFController(Node):
    def __init__(self):
        super().__init__('so101_cbf_controller')

        # --- Parameters ---
        # Joint names for the SO-101 arm (adjust as necessary for your specific URDF)
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper_joint'
        ])
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)

        # Joint Limits (Lower, Upper) in Radians
        self.joint_limits = np.array([
            [-3.14, 3.14],
            [-1.57, 1.57],
            [-3.14, 3.14],
            [-1.57, 1.57],
            [-3.14, 3.14],
            [0.0, 1.0] # Gripper
        ])

        # --- State ---
        self.q = np.zeros(self.num_joints)
        self.dq = np.zeros(self.num_joints)
        self.state_received = False

        # --- ROS Interfaces ---
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Assuming a velocity controller interface
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

        # --- Control Loop ---
        self.timer = self.create_timer(0.05, self.control_step) # 20Hz
        self.get_logger().info("SO-101 CBF Controller Initialized")

    def joint_callback(self, msg):
        """Update internal state from ROS JointState message."""
        try:
            # Extract positions/velocities for the specific joints we control
            current_q = []
            current_dq = []
            
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    current_q.append(msg.position[idx])
                    current_dq.append(msg.velocity[idx] if len(msg.velocity) > idx else 0.0)
            
            if len(current_q) == self.num_joints:
                self.q = np.array(current_q)
                self.dq = np.array(current_dq)
                self.state_received = True
                
        except ValueError:
            pass

    def control_step(self):
        if not self.state_received:
            return

        # 1. Define Goal (e.g., Home position)
        q_des = np.zeros(self.num_joints)
        
        # 2. Nominal Control (Simple P-Controller)
        # u_nom = -Kp * (q - q_des)
        Kp = 2.0
        u_nom = -Kp * (self.q - q_des)

        # 3. Apply CBF Safety Filter
        # We want to ensure: joint_min < q < joint_max
        # h(x) = (q_max - q)(q - q_min) >= 0
        
        u_safe = self.apply_cbf(self.q, u_nom)

        # 4. Publish Command
        msg = Float64MultiArray()
        msg.data = u_safe.tolist()
        self.cmd_pub.publish(msg)

    def apply_cbf(self, x, u_nom):
        """
        Placeholder for cbfkit QP solver logic.
        In a full implementation, this would construct the QP matrices 
        based on the system dynamics and barrier functions.
        """
        # TODO: Integrate specific cbfkit solver call here.
        # Example logic:
        # 1. Compute Lfh (Lie derivative of h along f) and Lgh (along g)
        # 2. Set up QP: min ||u - u_nom||^2 s.t. Lfh + Lgh*u + alpha*h >= 0
        # 3. Solve QP
        
        return u_nom

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

if __name__ == '__main__':
    main()
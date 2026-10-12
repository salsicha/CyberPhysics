#!/usr/bin/env python3
"""odom -> base_link on /tf from the RACECAR Turret simulator's /odom.

Isaac Sim's bundled ROS 2 Jazzy libraries have no tf2_msgs, so
racecar_turret_isaac.py publishes the wheel odometry on /odom and this node
broadcasts it as the transform racecarneo_ros_bridge.py publishes itself
(odom -> base_link only; robot_state_publisher has the rest), with the
odometry's own stamp and frames.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__('racecar_turret_odom_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._odom, 20)

    def _odom(self, msg):
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

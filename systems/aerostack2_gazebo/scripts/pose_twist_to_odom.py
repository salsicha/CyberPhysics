#!/usr/bin/env python3
import os

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class PoseTwistToOdom(Node):
    def __init__(self):
        super().__init__('pose_twist_to_odom')
        self.pose = None
        self.twist = None
        self.output_frame = os.environ.get('ODOM_FRAME', 'odom')
        self.child_frame = os.environ.get('ODOM_CHILD_FRAME', 'base_link')
        self.pub = self.create_publisher(
            Odometry,
            os.environ.get('ODOM_TOPIC', '/drone_sim_0/sensor_measurements/odom'),
            10,
        )
        self.create_subscription(
            PoseStamped,
            os.environ.get('POSE_TOPIC', '/drone_sim_0/ground_truth/pose'),
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            os.environ.get('TWIST_TOPIC', '/drone_sim_0/ground_truth/twist'),
            self._on_twist,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Publishing odometry on {self.pub.topic_name} from ground-truth pose/twist"
        )

    def _on_pose(self, msg):
        self.pose = msg
        self._publish()

    def _on_twist(self, msg):
        self.twist = msg
        self._publish()

    def _publish(self):
        if self.pose is None or self.twist is None:
            return
        msg = Odometry()
        msg.header = self.pose.header
        msg.header.frame_id = self.output_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose = self.pose.pose
        msg.twist.twist = self.twist.twist
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseTwistToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

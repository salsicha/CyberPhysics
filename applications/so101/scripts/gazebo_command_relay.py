#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandRelay(Node):
    def __init__(self):
        super().__init__("so101_gazebo_command_relay")
        self.publisher = self.create_publisher(
            Float64MultiArray, "/so101_position_controller/commands", 10
        )
        self.subscription = self.create_subscription(
            Float64MultiArray, "/so101/joint_commands", self.publisher.publish, 10
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommandRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

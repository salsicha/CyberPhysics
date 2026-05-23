#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import sys
import os

# Ensure Depth Anything V2 is in path
sys.path.append('/opt/Depth-Anything-V2')
try:
    from depth_anything_v2.dpt import DepthAnythingV2
except ImportError as e:
    print(f"Error importing DepthAnythingV2: {e}")
    DepthAnythingV2 = None

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_v2_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/depth_anything/depth')
        self.declare_parameter('encoder', 'vits')

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.encoder = self.get_parameter('encoder').value

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")

        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        self.get_logger().info(f"Loading model with encoder: {self.encoder}...")
        
        if DepthAnythingV2 is None:
            self.get_logger().error("DepthAnythingV2 class not found. Make sure the repository is cloned and in PYTHONPATH.")
            return

        self.model = DepthAnythingV2(**model_configs[self.encoder])
        
        # Load weights
        model_path = f"/opt/Depth-Anything-V2/checkpoints/depth_anything_v2_{self.encoder}.pth"
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model weights not found at {model_path}")
            return

        try:
            self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
            self.model = self.model.to(self.device).eval()
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {model_path}: {e}")
            raise e

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(Image, depth_topic, 10)
        self.get_logger().info(f"Subscribed to {image_topic}, Publishing to {depth_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        with torch.no_grad():
            depth = self.model.infer_image(cv_image)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = msg.header
        self.publisher.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

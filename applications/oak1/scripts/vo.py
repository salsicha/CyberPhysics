#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import os

"""
Relative Depth Estimation using Oak-1 (Monocular)
Uses a Neural Network (MiDaS) to estimate relative depth from a single RGB camera.
Publishes depth and RGB images as ROS topics.
"""

class RelativeDepthPublisher(Node):
    def __init__(self):
        super().__init__('relative_depth_publisher')
        self.pub_depth = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.pub_rgb = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        
        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        except ImportError:
            self.bridge = None
            self.get_logger().warn("CvBridge not found. Image publishing might fail or require manual conversion.")

    def publish_images(self, depth_frame, rgb_frame):
        if self.bridge:
            # Publish Depth (32FC1) - Relative depth (inverse depth usually)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="32FC1")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "oak_rgb_camera_optical_frame"
            self.pub_depth.publish(depth_msg)

            # Publish RGB
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
            rgb_msg.header.stamp = depth_msg.header.stamp
            rgb_msg.header.frame_id = "oak_rgb_camera_optical_frame"
            self.pub_rgb.publish(rgb_msg)

def get_blob_path():

    # https://blobconverter.luxonis.com/

    default_blob = "models/megadepth_8.blob"
    if os.path.exists(default_blob):
        return default_blob
        
    print(f"Model blob not found. Please download '{default_blob}' or install 'blobconverter'.")
    return None

def create_pipeline(blob_path):
    pipeline = dai.Pipeline()

    # Define sources and outputs
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    nn = pipeline.create(dai.node.NeuralNetwork)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_rgb = pipeline.create(dai.node.XLinkOut)

    xout_depth.setStreamName("depth")
    xout_rgb.setStreamName("rgb")

    # Properties
    cam_rgb.setPreviewSize(256, 256)
    cam_rgb.setInterleaved(False) # Planar for NN
    cam_rgb.setFps(20)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    
    # Neural Network
    nn.setBlobPath(blob_path)
    nn.setNumInferenceThreads(2)
    nn.input.setBlocking(False)

    # Linking
    cam_rgb.preview.link(nn.input)
    nn.out.link(xout_depth.input)
    cam_rgb.preview.link(xout_rgb.input)

    return pipeline

def main(args=None):
    rclpy.init(args=args)
    
    blob_path = get_blob_path()
    if not blob_path:
        rclpy.shutdown()
        return

    pipeline = create_pipeline(blob_path)
    node = RelativeDepthPublisher()

    with dai.Device(pipeline) as device:
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while rclpy.ok():
            in_depth = q_depth.tryGet()
            in_rgb = q_rgb.tryGet()

            if in_depth and in_rgb:
                # Process Depth
                # MiDaS output is 1x1x256x256
                layer_name = in_depth.getAllLayerNames()[0]
                depth_data = in_depth.getLayerFp16(layer_name)
                depth_frame = np.array(depth_data).reshape((256, 256)).astype(np.float32)
                
                # Process RGB
                rgb_frame = in_rgb.getFrame() # Shape (3, 256, 256) because Interleaved=False
                if rgb_frame.shape[0] == 3: 
                    rgb_frame = rgb_frame.transpose(1, 2, 0) # Convert to (256, 256, 3)
                
                # Publish
                node.publish_images(depth_frame, rgb_frame)

            rclpy.spin_once(node, timeout_sec=0.001)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

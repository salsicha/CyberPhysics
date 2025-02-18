#!/usr/bin/env python3

import sys
import time
from threading import Thread, Event

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
from std_msgs.msg import Header
from point_cloud2 import create_cloud_xyz32

class OakFeatureTracker(Node):
    def __init__(self):
        super().__init__('oak_feature_tracker')
        
        # Initialize the ROS 2 publisher for feature points
        self.publisher_ = self.create_publisher(PointCloud2, 'tracked_features', 10)
        
        # Create a timer to publish features at a fixed rate
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize DepthAI pipeline and components
        self.pipeline = dai.Pipeline()
        
        # Define the mono camera node
        self.camera = self.pipeline.create(dai.node.MonoCamera)
        self.camera.setResolution(dai.MonoCameraProperties.Resolution.THE_400_P)
        self.camera.setFps(30)
        
        # Link camera to feature tracker
        self.feature_tracker = self.pipeline.create(dai.node.FeatureTracker)
        self.camera.out.link(self.feature_tracker.inputImage)
        
        # Create output queue for features
        self.feature_queue = self.pipeline.create(dai.node.XLinkOut)
        self.feature_queue.setStreamName("feature")
        self.feature_tracker.trackedFeatures.link(self.feature_queue.input)
        
        # Start the pipeline
        self.device = dai.Device()
        self.device.startPipeline(self.pipeline)

    def timer_callback(self):
        try:
            features = self.device.getOutputQueue(name="feature", maxSize=1, blocking=False).get().trackedFeatures
            if features.numPoints > 0:
                points = []
                for point in features.points:
                    x = point.x
                    y = point.y
                    z = point.z
                    points.append([x, y, z])

                cloud_msg = create_cloud_xyz32(
                    Header(frame_id="camera_frame"),
                    points,
                )
                
                self.publisher_.publish(cloud_msg)
            else:
                self.get_logger().info("No tracked features available")
        except dai.DaiException as e:
            self.get_logger().error(f"DepthAI error: {e}")
        except Exception as e:
            self.get_logger().error(f"General error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    oak_feature_tracker = OakFeatureTracker()
    
    rclpy.spin(oak_feature_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oak_feature_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
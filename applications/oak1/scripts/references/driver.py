#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai
import time 
import argparse
import threading
import json
import numpy as np

class OakCameraNode(Node):
    def __init__(self):
        '''
        Node for publishing raw image from Oak-1 camera
        '''

        super().__init__('oak_camera_node')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, f'{TOPIC_NAMESPACE}/image_raw', 10)

        # Specify device IP
        self.device_info = dai.DeviceInfo(DEVICE_IP)
        
        # Create main pipeline
        self.pipeline = dai.Pipeline()

        # Create video pipeline
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.camRgb.setImageOrientation(dai.CameraImageOrientation.NORMAL)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        self.camRgb.setIspScale(1,5)
        self.camRgb.setFps(RATE)

        # Create xlinkout pipeline
        self.xoutVideo1 = self.pipeline.create(dai.node.XLinkOut)
        self.xoutVideo1.setStreamName("video")
        self.camRgb.video.link(self.xoutVideo1.input)        

        if custom_script is not None:
            # Create script pipeline
            self.script = self.pipeline.create(dai.node.Script)
            self.script.setProcessor(dai.ProcessorType.LEON_CSS)
            self.script.setScript(custom_script)

            self.xoutVideo2 = self.pipeline.create(dai.node.XLinkOut)
            self.xoutVideo2.setStreamName('end')
            self.script.outputs['end'].link(self.xoutVideo2.input)

        if CALIBRATION_PATH is not None:
            self.calibration_params = self.load_calibration_parameters(CALIBRATION_PATH)
            self.dist_coeffs = np.array(self.calibration_params["distortion_coefficients"])
            self.K = np.array(self.calibration_params["camera_matrix"])
            self.get_logger().info("Calibration parameters successfully loaded")
        else:
            self.calibration_params = None

        while rclpy.ok():
            try:
                self.device = dai.Device(self.pipeline, self.device_info)	
                self.get_logger().info("Oak-1 initialized")
                break
            except RuntimeError:
                self.get_logger().info("Failed to connect to device, trying again...")
                time.sleep(0.1)

    def load_calibration_parameters(self, calibration_json_path):
        """
        Load camera calibration parameters from a JSON file.

        Args:
            calibration_json_path (str): Path to the JSON file containing calibration data.

        Returns:
            dict: A dictionary containing the calibration parameters.
        """
        
        try:
            with open(calibration_json_path, 'r') as json_file:
                calibration_params = json.load(json_file)
            return calibration_params
        except FileNotFoundError:
            self.get_logger().info(f"Calibration JSON file not found at {calibration_json_path}.")
            return None
        except json.JSONDecodeError as e:
            self.get_logger().info(f"Error decoding JSON: {e}")
            return None

    def publish_oak1_image(self):
        '''
        Thread for publishing raw image from Oak-1 camera
        '''
        video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

        while rclpy.ok():
            videoIn = video.tryGet()  

            if videoIn is not None:
                img = videoIn.getCvFrame()
                if self.calibration_params is not None:
                    img = cv2.undistort(img, self.K, self.dist_coeffs)

                # Populating img data field
                img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                img_msg.header = Header()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'oak1_camera'

                self.img_pub.publish(img_msg)
                
                if DEBUG:
                    cv2.imshow(TOPIC_NAMESPACE, videoIn.getCvFrame())
                    cv2.waitKey(1)
               
    def run_custom_script(self):
        '''
        Thread for outputting logs from custom script on Oak-1 camera
        '''
        if custom_script is not None:
            self.device.getOutputQueue("end").get()

def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    
    try:
        # Create and start the image display thread
        image_publish_thread = threading.Thread(target=node.publish_oak1_image)
        image_publish_thread.start()
        
        # Create and start the custom script thread
        custom_script_thread = threading.Thread(target=node.run_custom_script)
        custom_script_thread.start()

        rclpy.spin(node)

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    # Args
    parser = argparse.ArgumentParser(description="Simple ROS driver for Oak-1 camera. Publishes a scaled down 12MP image and allows for execution of custom scripts in the camera's onboard compute", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--topic-namespace", nargs='?', default="oak1", help="ROS topic name")
    parser.add_argument("--device-ip", nargs='?', default="169.254.1.222", help="Device IP address")
    parser.add_argument("--script-file", nargs='?', default=None, help="Path to python file to be ran in Oak-1 camera's onboard compute")
    parser.add_argument("--calibration-file", nargs='?', default=None, help="Path to camera intrinsic calibration")
    parser.add_argument("--rate", nargs='?', default="30", help="Frame rate of camera")
    parser.add_argument("--debug", action='store_true', help="Enable debug visualization")

    args = parser.parse_args()

    TOPIC_NAMESPACE = args.topic_namespace
    DEVICE_IP = args.device_ip
    SCRIPT_PATH = args.script_file
    CALIBRATION_PATH = args.calibration_file
    RATE = int(args.rate)
    DEBUG = args.debug

    # Reading custom script
    if SCRIPT_PATH is not None:
        with open(SCRIPT_PATH, "r") as script_file:
            custom_script = script_file.read()
    else:
        custom_script = None
        
    main()

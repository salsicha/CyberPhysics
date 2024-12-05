#!/usr/bin/env python3

"""
Test the commonly used camera settings.
It assumes the required ROS2 nodes are up and running
"""

import rclpy
import time
import pytest

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from flir_camera_msgs.msg import CameraInfoExt

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters


CAMERA_NAME = "blackfly_1"


@pytest.fixture(scope="session")
def ros_node():
    rclpy.init()
    yield ROSGetParam()
    rclpy.shutdown()

class FrameDrop(Node):
    """Run the camera for X seconds and check for frame drops."""

    def __init__(self):
        super().__init__("test_camera_frame_drop")

        self.max_frame_drop = 1
        self.max_duration_secs = 10

        self.start_frame_drop = None
        self.end_frame_drop = None

    def callback_image(self, msg):
        assert msg.width > 0
        assert msg.height > 0

    def callback_camera_info_ext(self, msg):
        if self.start_frame_drop is None:
            self.start_frame_drop = msg.frames_dropped
        else:
            self.end_frame_drop = msg.frames_dropped

    def run(self):
        self.create_subscription(Image, f"/{CAMERA_NAME}/image_raw", self.callback_image, 1)
        self.create_subscription(CameraInfoExt, f"/{CAMERA_NAME}/camera_info_ext", self.callback_camera_info_ext, 1)

        self.get_logger().info("Collecting frames ...")
        start = time.time()

        while rclpy.ok() and time.time() - start < self.max_duration_secs:
            rclpy.spin_once(self, timeout_sec=1.0)

        assert self.start_frame_drop is not None, "No image data"
        assert self.end_frame_drop is not None, "No image data"
        assert self.end_frame_drop - self.start_frame_drop <= self.max_frame_drop, f"{self.end_frame_drop - self.start_frame_drop} < {self.max_frame_drop}"


class ROSGetParam(Node):
    """Check if param exists"""
    TIMEOUT = 5.0

    def __init__(self):
        super().__init__("test_camera_get_param")

        self.wait_time_sec = 0.1
        self.cli = self.create_client(GetParameters, f'/{CAMERA_NAME}/get_parameters')

        while not self.cli.wait_for_service(timeout_sec=ROSGetParam.TIMEOUT):
            assert False, f"/{CAMERA_NAME}/get_parameters not available!"

        self.req = GetParameters.Request()

    def get(self, param_name):
        self.req.names = [param_name]
        future = self.cli.call_async(self.req)

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if future.done():
                break

            if time.time() - start > ROSGetParam.TIMEOUT:
                break

        assert future.done(), "param get failed"

        msg = future.result()
        assert len(msg.values) > 0, "no values returned"

def test_frame_drop(ros_node):
    FrameDrop().run()

def test_auto_exposure_upper_limit(ros_node):
    ros_node.get("auto_exposure_time_upper_limit")

def test_exposure_metering_mode(ros_node):
    ros_node.get("exposure_metering_mode")

def test_image_compression_mode(ros_node):
    ros_node.get("image_compression_mode")

def test_device_link_throughput_limit(ros_node):
    ros_node.get("device_link_throughput_limit")


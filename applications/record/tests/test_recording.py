#!/usr/bin/env python3

"""
Platform specific test.

These test should be run after deploying the specific system tarball.
"""

import pytest
import rclpy
import time
import rosbags
import yaml
import argparse
import os
import io
import psutil

from glob import glob
from pathlib import Path
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, SetBool
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, Stores, get_typestore


def register_custom_msg():
    # Register all messages we need to read the rosbag
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    add_types = {}
    for custom_msg in ["novatel_oem7_msgs", "gps_msgs", "flir_camera_msgs"]:
        for f in glob(f"/opt/ros/humble/share/{custom_msg}/msg/*.msg"):
            text = Path(f).read_text()
            stem = Path(f).stem
            add_types.update(get_types_from_msg(text, f"{custom_msg}/msg/{stem}"))

    typestore.register(add_types)

    return typestore

typestore = register_custom_msg()


def check_topic_and_rate(bag_path, topic_rate_tol):
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = []
        for x in reader.connections:
            if x.topic in topic_rate_tol:
                assert x.msgcount > 0, f"No messages for topic {x.topic} found in {bag_path}"
                connections.append(x)

        # find start and end timestamp for each topic
        start = {}
        end = {}

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            if conn.topic not in start:
                start[conn.topic] = timestamp
            else:
                end[conn.topic] = timestamp

        for x in reader.connections:
            if x.topic in topic_rate_tol:
                duration = (end[x.topic] - start[x.topic])*1e-9
                actual_rate = x.msgcount / duration
                expected_rate, tol = topic_rate_tol[x.topic]

                # allow faster than expected rate
                min_rate = expected_rate - tol
                assert actual_rate > min_rate, f"{x.topic} expected_rate={expected_rate:.2f} actual_rate={actual_rate:.2f} "


def check_topic_exists(bag_path, topics):
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        bag_topics = []
        for x in reader.connections:
            bag_topics.append(x.topic)

        for t in topics:
            assert t in bag_topics, f"topic {t} not found in {bag_path}"

    return


def get_camera_info_ext(bag_path, topic):
    """Returns all the CameraInfoExt message"""

    ret = []
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = []
        for x in reader.connections:
            if x.topic == topic:
                assert x.msgcount > 0, f"No messages for topic {topic} found in {bag_path}"
                connections.append(x)

        assert len(connections) == 1, f"topic {topic} not found in {bag_path}"

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            ret.append(msg)

    return ret


def get_camera_pixel_encoding(bag_path, topic):
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = []
        for x in reader.connections:
            if x.topic == topic:
                assert x.msgcount > 0, f"No messages for topic {topic} found in {bag_path}"
                connections.append(x)

        assert len(connections) == 1, f"topic {topic} not found in {bag_path}"

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            return msg.encoding


def timestamp_from_header(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def check_dropped_frame_rate(bag_path, camera_name, max_frame_drop_rate):
    camera_info = get_camera_info_ext(bag_path, f"/{camera_name}/camera_info_ext")
    frames_dropped = camera_info[-1].frames_dropped - camera_info[0].frames_dropped
    duration = timestamp_from_header(camera_info[-1]) - timestamp_from_header(camera_info[0])
    frame_drop_rate = frames_dropped / duration

    assert frame_drop_rate < max_frame_drop_rate, f"frames_dropped={frames_dropped}"


def run_test_on_bag(bag_path, param):
    max_frame_drop_rate = 0.0005
    imu_tol = 4.0 # can sometimes get slightly higher rate than 100hz
    camera_tol = 3.0

    # rate determined on the fly
    topic_rate_tol = {
        "/novatel/oem7/imu/data_raw": (-1, -1),
        "/blackfly_1/image_raw": (-1, -1),
        "/blackfly_2/image_raw": (-1, -1),
    }

    # get Novatel rate
    novatel_rate = param["/novatel/oem7/main"]["rate"]
    topic_rate_tol["/novatel/oem7/imu/data_raw"] = (novatel_rate, imu_tol)

    # get camera rate
    for camera in ["blackfly_1", "blackfly_2"]:
        frame_rate = param[f"/{camera}"]["frame_rate"]
        topic_rate_tol[f"/{camera}/image_raw"] = (frame_rate, camera_tol)

    check_topic_and_rate(bag_path, topic_rate_tol)
    check_dropped_frame_rate(bag_path, "blackfly_1", max_frame_drop_rate)
    check_dropped_frame_rate(bag_path, "blackfly_2", max_frame_drop_rate)

    # check camera is color
    encoding = get_camera_pixel_encoding(bag_path, "/blackfly_1/image_raw")
    assert encoding == "bayer_rggb8"

    encoding = get_camera_pixel_encoding(bag_path, "/blackfly_2/image_raw")
    assert encoding == "bayer_rggb8"


class RecordBag(Node):
    def __init__(self):
        super().__init__("recording_test")

        self.flight_pub = self.create_publisher(String, "/flight_number", 10)
        self.start_cli = self.get_service("/start_recording", Trigger)
        self.stop_cli = self.get_service("/stop_recording", Trigger)
        self.blackfly_1_cli = self.get_service("/blackfly_1/get_parameters", GetParameters)
        self.blackfly_2_cli = self.get_service("/blackfly_2/get_parameters", GetParameters)
        self.novatel_cli = self.get_service("/novatel/oem7/main/get_parameters", GetParameters)

    def set_flight(self, flight_id):
        timeout_sec = 10.0
        start = time.time()

        while rclpy.ok():
            if self.flight_pub.get_subscription_count() > 0:
                # publish a few times to increase chance of working
                for i in range(10):
                    msg = String()
                    msg.data = flight_id
                    self.flight_pub.publish(msg)

                break

            elapsed = time.time() - start

            assert elapsed < timeout_sec, "could not set flight id, no subscriber"

    def get_service(self, service, msg_type):
        timeout_sec = 10.0

        cli = self.create_client(msg_type, service)

        start = time.time()
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{service}, waiting for service to be available ...")
            elapsed = time.time() - start
            assert elapsed < timeout_sec, f"can't find service {service}"

        return cli

    def send_request(self, client, req, attempts=1, timeout_sec=30):
        for i in range(attempts):
            self.get_logger().info(f"send_request: attempt={i+1}/{attempts}, timeout_sec={timeout_sec}, req={req}")

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            res = future.result()

            if res is not None:
                break

        assert res is not None, f"timeout calling {req}"

        self.get_logger().info(f"send_request: {req}")

        return res

    def get_param(self, cli, names):
        req = GetParameters.Request()
        req.names = names

        msg = self.send_request(cli, req)
        assert len(msg.values) > 0
        return msg.values

    def start_recording(self):
        msg = self.send_request(self.start_cli, Trigger.Request())

        if msg.success is False:
            if "Errno 17" in msg.message:
                self.get_logger().warning(f"start_recording failed with: {msg}, will remove path and try again")
                path = msg.message.split("'")[1]
                os.system(f"rm -rf {path}")

                msg = self.send_request(self.start_cli, Trigger.Request())

        assert msg.success, msg.message
        return msg

    def stop_recording(self):
        msg = self.send_request(self.stop_cli, Trigger.Request())
        assert msg.success, msg.message
        return msg

    def record(self, bag_duration_sec):
        param = {
            "/blackfly_1" : {},
            "/blackfly_2" : {},
            "/novatel/oem7/main" : {}
        }

        param["/blackfly_1"]["frame_rate"] = self.get_param(self.blackfly_1_cli, ["frame_rate"])[0].integer_value
        param["/blackfly_2"]["frame_rate"] = self.get_param(self.blackfly_2_cli, ["frame_rate"])[0].integer_value
        param["/novatel/oem7/main"]["rate"] = self.get_param(self.novatel_cli, ["supported_imus.68.rate"])[0].integer_value

        start_msg = self.start_recording()
        time.sleep(bag_duration_sec)
        self.stop_recording()

        return start_msg.message, param


def test_FTB_config_60_sec_bag():
    bag_duration_sec = 60.0

    rclpy.init()

    record_bag = RecordBag()
    record_bag.set_flight("test")

    bag_path, param = record_bag.record(bag_duration_sec)
    assert len(bag_path) > 0

    run_test_on_bag(bag_path, param)
    os.system(f"rm -rf {bag_path}")

    rclpy.shutdown()

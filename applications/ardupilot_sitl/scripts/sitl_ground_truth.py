#!/usr/bin/env python3
"""Publish ArduPilot SITL physics truth as ROS ground-truth topics.

ArduPilot's built-in SITL physics reports the true vehicle state in the
MAVLink SIM_STATE message, independent of the simulated GPS (which can be
degraded or disabled with SIM_GPS_* parameters). MAVROS' sim_state plugin
publishes it once the message is streamed; this node requests the stream
and republishes it under the vehicle's simulator-only ``ground_truth``
namespace:

- ``<ns>/ground_truth/fix``: NavSatFix, true latitude/longitude and true
  altitude above mean sea level (SIM_STATE alt; *not* the ellipsoidal
  altitude MAVROS reports for the GPS), status STATUS_FIX;
- ``<ns>/ground_truth/odom``: Odometry, true attitude (ENU/FLU) and
  position in local ENU metres about ``origin_lat``/``origin_lon`` (z about
  ``origin_alt``, AMSL), with the synthetic-world projection every
  navigation node uses.

Simulator-only: never run this on hardware, where SIM_STATE does not exist.
"""
from __future__ import annotations

import math

import rclpy
from mavros_msgs.srv import MessageInterval
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from synthetic_world import latlon_to_local

MAVLINK_MSG_ID_SIM_STATE = 108


def valid_truth(msg):
    return (all(math.isfinite(value) for value in (
        msg.latitude, msg.longitude, msg.altitude))
        and -90.0 <= msg.latitude <= 90.0 and -180.0 <= msg.longitude <= 180.0
        and not (msg.latitude == 0.0 and msg.longitude == 0.0))


class SitlGroundTruth(Node):
    def __init__(self):
        super().__init__('sitl_ground_truth')
        defaults = {
            'mavros_namespace': '/mavros',
            'namespace': 'aerodrone',
            'rate_hz': 20.0,
            'origin_lat': 0.0,
            'origin_lon': 0.0,
            'origin_alt': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        mavros = '/' + str(self.get_parameter('mavros_namespace').value).strip('/')
        prefix = '/' + str(self.get_parameter('namespace').value).strip('/')
        self.origin = tuple(float(self.get_parameter(name).value)
                            for name in ('origin_lat', 'origin_lon', 'origin_alt'))
        self.attitude = None
        self.received = False
        self.fix_pub = self.create_publisher(
            NavSatFix, f'{prefix}/ground_truth/fix', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(
            Odometry, f'{prefix}/ground_truth/odom', qos_profile_sensor_data)
        self.create_subscription(
            Imu, f'{mavros}/sim_state/attitude', self._on_attitude,
            qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, f'{mavros}/sim_state/global_position', self._on_position,
            qos_profile_sensor_data)
        self.interval_client = self.create_client(
            MessageInterval, f'{mavros}/set_message_interval')
        self.create_timer(2.0, self._request_stream)
        self.get_logger().info(f'SITL ground truth -> {prefix}/ground_truth/*')

    def _request_stream(self):
        # Re-request until truth flows (SITL may restart or reset streams).
        if self.received or not self.interval_client.service_is_ready():
            self.received = False
            return
        request = MessageInterval.Request()
        request.message_id = MAVLINK_MSG_ID_SIM_STATE
        request.message_rate = float(self.get_parameter('rate_hz').value)
        self.interval_client.call_async(request)

    def _on_attitude(self, msg):
        self.attitude = msg

    def _on_position(self, msg):
        if not valid_truth(msg):
            return
        self.received = True
        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = 'earth'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = (
            msg.latitude, msg.longitude, msg.altitude)
        fix.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)
        if self.attitude is None:
            return
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'ground_truth'
        odom.child_frame_id = 'base_link'
        lat0, lon0, alt0 = self.origin
        if lat0 == 0.0 and lon0 == 0.0:
            lat0, lon0 = msg.latitude, msg.longitude
            self.origin = (lat0, lon0, alt0)
        east, north = latlon_to_local(msg.latitude, msg.longitude, lat0, lon0)
        odom.pose.pose.position.x = float(east)
        odom.pose.pose.position.y = float(north)
        odom.pose.pose.position.z = float(msg.altitude - alt0)
        odom.pose.pose.orientation = self.attitude.orientation
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = SitlGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

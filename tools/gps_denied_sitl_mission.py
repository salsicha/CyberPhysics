#!/usr/bin/env python3
"""Fly the Aerodrone GPS-denied SITL mission and record telemetry.

Runs inside the salsicha/cyberphysics:aerostack2-latest image against the stack in
tools/gps_denied_sitl.yaml (see tools/gps_denied_sitl_test.sh):

1. waits for MAVROS, a GPS fix, the EKF origin and the fused navigation fix;
2. GUIDED, arm, take off, then flies a square of local position setpoints;
3. after GPS_DENIED_DENY_AFTER_S at altitude disables the simulated GPS
   (SIM_GPS_DISABLE=1) for GPS_DENIED_DENIED_S, then restores it for
   GPS_DENIED_RESTORE_S (GPS_DENIED_DENIED_S=0: never disables it, a normal
   flight of GPS_DENIED_DENY_AFTER_S + GPS_DENIED_RESTORE_S);
4. records 1 Hz samples of simulator ground truth, the fused navigation fix,
   ArduPilot's own (EKF) position, the correction source, flight mode and
   autopilot status texts, and writes them as JSON for
   tools/gps_denied_sitl_scorer.py.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW, State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, ParamSetV2, SetMode
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

MESSAGE_RATES_HZ = {24: 5.0, 30: 20.0, 31: 20.0, 32: 20.0, 33: 5.0, 74: 5.0,
                    245: 2.0, 253: 2.0}


def env(name, default):
    return float(os.environ.get(name, default))


def fix_tuple(msg):
    if msg is None:
        return None
    return [msg.latitude, msg.longitude, msg.altitude]


class Mission(Node):
    def __init__(self, namespace):
        super().__init__('gps_denied_sitl_mission')
        self.ns = namespace
        self.state = None
        self.latest = {}
        self.texts = []
        self.origin = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(State, f'{namespace}/state', self._state, 10)
        self.create_subscription(StatusText, f'{namespace}/statustext/recv', self._text,
                                 qos_profile_sensor_data)
        self.create_subscription(GeoPointStamped, '/navigation/origin', self._origin, latched)
        for key, msg_type, topic in (
                ('truth', NavSatFix, '/aerodrone/ground_truth/fix'),
                ('fused', NavSatFix, '/navigation/fix'),
                ('ekf', NavSatFix, f'{namespace}/global_position/global'),
                ('rel_alt', Float64, f'{namespace}/global_position/rel_alt'),
                ('gps1', GPSRAW, f'{namespace}/gpsstatus/gps1/raw')):
            self.create_subscription(
                msg_type, topic, lambda msg, key=key: self.latest.__setitem__(key, msg),
                qos_profile_sensor_data)
        self.create_subscription(
            String, '/navigation/correction_source',
            lambda msg: self.latest.__setitem__('source', msg.data), 10)
        self.create_subscription(
            PoseStamped, f'{namespace}/local_position/pose',
            lambda msg: self.latest.__setitem__('pose', msg), qos_profile_sensor_data)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, f'{namespace}/setpoint_position/local', 10)
        self.service_clients = {
            'mode': self.create_client(SetMode, f'{namespace}/set_mode'),
            'arm': self.create_client(CommandBool, f'{namespace}/cmd/arming'),
            'takeoff': self.create_client(CommandTOL, f'{namespace}/cmd/takeoff'),
            'interval': self.create_client(MessageInterval, f'{namespace}/set_message_interval'),
            'param': self.create_client(ParamSetV2, f'{namespace}/param/set'),
        }
        self.start = time.monotonic()

    def _state(self, msg):
        self.state = msg

    def _text(self, msg):
        self.texts.append({'t': round(time.monotonic() - self.start, 2), 'text': msg.text})
        self.get_logger().info(f'FCU: {msg.text}')

    def _origin(self, msg):
        self.origin = (msg.position.latitude, msg.position.longitude, msg.position.altitude)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, name, request, timeout=10.0):
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {name} unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def set_param(self, name, value):
        request = ParamSetV2.Request()
        request.force_set = True
        request.param_id = name
        request.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        response = self.call('param', request)
        if response is None or not response.success:
            raise RuntimeError(f'could not set {name}={value}')

    def wait_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f'timed out waiting for {what}')

    def setpoint(self, east, north, up):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = east, north, up
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/mavros')
    parser.add_argument('--output', default='/out/telemetry.json')
    parser.add_argument('--ready-timeout', type=float, default=600.0)
    args = parser.parse_args()
    takeoff_alt = env('GPS_DENIED_TAKEOFF_ALT_M', 150.0)
    leg = env('GPS_DENIED_LEG_M', 300.0)
    deny_after = env('GPS_DENIED_DENY_AFTER_S', 60.0)
    denied_s = env('GPS_DENIED_DENIED_S', 300.0)
    restore_s = env('GPS_DENIED_RESTORE_S', 30.0)

    rclpy.init()
    node = Mission(args.namespace)
    samples = []
    result = {'config': {'takeoff_alt_m': takeoff_alt, 'leg_m': leg,
                         'deny_after_s': deny_after, 'denied_s': denied_s,
                         'restore_s': restore_s,
                         'gps_input_corrections': os.environ.get('GPS_INPUT_CORRECTIONS', '')},
              'samples': samples, 'events': []}

    def event(name):
        result['events'].append({'t': round(time.monotonic() - node.start, 2), 'event': name})
        node.get_logger().warn(f'EVENT {name}')

    try:
        node.wait_until(lambda: node.state is not None and node.state.connected,
                        args.ready_timeout, 'MAVROS connection')
        for message_id, rate in MESSAGE_RATES_HZ.items():
            request = MessageInterval.Request()
            request.message_id, request.message_rate = message_id, rate
            node.call('interval', request)
        node.wait_until(
            lambda: (node.latest.get('ekf') is not None and node.latest['ekf'].status.status >= 0
                     and node.origin is not None and 'fused' in node.latest
                     and 'truth' in node.latest and 'pose' in node.latest),
            args.ready_timeout, 'GPS, EKF origin, fused fix and ground truth')
        node.spin_for(5.0)
        event('ready')
        mode = SetMode.Request()
        mode.custom_mode = 'GUIDED'
        if not node.call('mode', mode).mode_sent:
            raise RuntimeError('GUIDED rejected')
        arm = CommandBool.Request()
        arm.value = True
        if not node.call('arm', arm).success:
            raise RuntimeError('arming rejected')
        takeoff = CommandTOL.Request()
        takeoff.altitude = takeoff_alt
        if not node.call('takeoff', takeoff).success:
            raise RuntimeError('takeoff rejected')
        event('takeoff')
        node.wait_until(lambda: 'rel_alt' in node.latest and
                        node.latest['rel_alt'].data >= 0.95 * takeoff_alt, 180.0, 'takeoff altitude')
        event('at_altitude')

        square = [(leg / 2, -leg / 2), (leg / 2, leg / 2), (-leg / 2, leg / 2), (-leg / 2, -leg / 2)]
        waypoint = 0
        phase = 'gps'
        t_alt = time.monotonic()
        last_sample = 0.0
        left_guided = False
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            elapsed = now - t_alt
            if phase == 'gps' and elapsed >= deny_after and denied_s <= 0:
                phase = 'restored'  # normal flight: GPS is never denied
            elif phase == 'gps' and elapsed >= deny_after:
                node.set_param('SIM_GPS_DISABLE', 1)
                phase = 'denied'
                event('gps_disabled')
            elif phase == 'denied' and elapsed >= deny_after + denied_s:
                node.set_param('SIM_GPS_DISABLE', 0)
                phase = 'restored'
                event('gps_restored')
            elif phase == 'restored' and elapsed >= deny_after + denied_s + restore_s:
                break
            pose = node.latest.get('pose')
            east, north = square[waypoint % len(square)]
            if pose is not None and math.hypot(pose.pose.position.x - east,
                                               pose.pose.position.y - north) < 10.0:
                waypoint += 1
                east, north = square[waypoint % len(square)]
            node.setpoint(east, north, takeoff_alt)
            if now - last_sample >= 1.0:
                last_sample = now
                fused = node.latest.get('fused')
                samples.append({
                    't': round(now - node.start, 2),
                    'phase': phase,
                    'mode': node.state.mode if node.state else None,
                    'armed': bool(node.state.armed) if node.state else None,
                    'truth': fix_tuple(node.latest.get('truth')),
                    'fused': fix_tuple(fused),
                    'fused_status': int(fused.status.status) if fused else None,
                    'fused_variance_m2': float(fused.position_covariance[0]) if fused else None,
                    'ekf': fix_tuple(node.latest.get('ekf')),
                    'correction_source': node.latest.get('source', ''),
                    'gps1_fix_type': int(node.latest['gps1'].fix_type) if 'gps1' in node.latest else None,
                    'waypoint': waypoint,
                })
            if node.state is not None and (not node.state.armed or node.state.mode != 'GUIDED'):
                if not left_guided:
                    left_guided = True
                    event(f'left_guided:{node.state.mode}:armed={node.state.armed}')
                # Keep recording what the vehicle does after leaving GUIDED.
                if not node.state.armed:
                    break
        result['completed'] = phase == 'restored'
    except Exception as exc:  # recorded for the scorer, then re-raised
        result['error'] = str(exc)
        node.get_logger().error(str(exc))
        raise
    finally:
        result['status_texts'] = node.texts
        result['origin'] = node.origin
        with open(args.output, 'w') as stream:
            json.dump(result, stream)
        node.get_logger().info(f'Wrote {len(samples)} samples to {args.output}')
        try:
            land = SetMode.Request()
            land.custom_mode = 'LAND'
            node.call('mode', land, timeout=3.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

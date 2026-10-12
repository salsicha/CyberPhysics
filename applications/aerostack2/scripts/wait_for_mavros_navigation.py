#!/usr/bin/env python3
"""Configure telemetry and gate autonomy until the vehicle is ready to fly.

Ready means MAVROS is connected with a GPS fix (unless --no-require-gps,
for a vehicle that may start GPS-denied) and finite local odometry and,
with --behavior-tree, that every Aerostack2 server the mission tree
calls can be discovered. as2_behavior_tree builds every node of the tree
before its first tick, and each action node then waits only 1 s for its
action server (BtActionNode::createActionClient in as2_behavior_tree 1.1.3;
server_timeout does not apply to that wait). A tree started before the
behaviour servers are up aborts.
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import sys
import time
from xml.etree import ElementTree

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Required by MAVROS' sensor, GPS, local-position, and velocity plugins.
MESSAGE_RATES_HZ = {
    24: 5.0,   # GPS_RAW_INT
    30: 20.0,  # ATTITUDE
    31: 20.0,  # ATTITUDE_QUATERNION
    32: 20.0,  # LOCAL_POSITION_NED
    33: 5.0,   # GLOBAL_POSITION_INT
    74: 5.0,   # VFR_HUD
    147: 2.0,  # BATTERY_STATUS
    245: 2.0,  # EXTENDED_SYS_STATE
}

# as2_behavior_tree 1.1.3 action nodes: their action server (as2_names,
# relative to the tree's namespace; a server_name port replaces it) and type.
BEHAVIOR_ACTIONS = {
    "TakeOff": ("TakeoffBehavior", "as2_msgs/action/Takeoff"),
    "GoTo": ("GoToBehavior", "as2_msgs/action/GoToWaypoint"),
    "Land": ("LandBehavior", "as2_msgs/action/Land"),
    "FollowPath": ("FollowPathBehavior", "as2_msgs/action/FollowPath"),
}
# Service nodes: the service is the node's service_name port.
BEHAVIOR_SERVICES = {
    "Arm": "std_srvs/srv/SetBool",
    "Disarm": "std_srvs/srv/SetBool",
    "Offboard": "std_srvs/srv/SetBool",
}


def resolve_name(name, namespace):
    """A ROS name as a node in `namespace` resolves it."""
    if name.startswith("/"):
        return name
    namespace = "/" + namespace.strip("/")
    return f"{namespace.rstrip('/')}/{name}"


def behavior_tree_servers(tree_xml, namespace):
    """Action servers and services the tree's nodes connect to, as
    ({name: action type}, {name: service type}). Only the trees count, not
    a Groot TreeNodesModel; blackboard-remapped names cannot be known here."""
    actions, services = {}, {}
    for tree in ElementTree.fromstring(tree_xml).iter("BehaviorTree"):
        for element in tree.iter():
            node_id = element.get("ID") if element.tag == "Action" else element.tag
            if node_id in BEHAVIOR_ACTIONS:
                default_name, type_name = BEHAVIOR_ACTIONS[node_id]
                name = element.get("server_name", default_name)
                if not name.startswith("{"):
                    actions[resolve_name(name, namespace)] = type_name
            elif node_id in BEHAVIOR_SERVICES:
                name = element.get("service_name", "")
                if name and not name.startswith("{"):
                    services[resolve_name(name, namespace)] = BEHAVIOR_SERVICES[node_id]
    return actions, services


def interface_type(type_name):
    """The Python class of a ROS interface such as 'std_srvs/srv/SetBool'."""
    package, kind, name = type_name.split("/")
    return getattr(importlib.import_module(f"{package}.{kind}"), name)


def request_message_interval(node, client, message_id, rate_hz, timeout_sec=5.0):
    """Ask the autopilot to send MAVLink message `message_id` at `rate_hz`
    (MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS's set_message_interval
    service, `client`); True once it accepted. Also used by
    mavros_message_rates.py, which keeps the rates of the BlueOS vehicles."""
    request = MessageInterval.Request()
    request.message_id = message_id
    request.message_rate = rate_hz
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    response = future.result()
    return response is not None and bool(response.success)


class NavigationReadiness(Node):
    def __init__(self, namespace, actions=None, services=None, require_gps=True):
        super().__init__("navigator_navigation_readiness")
        self.require_gps = require_gps
        self.state = None
        self.fix = None
        self.odom = None
        self.intervals_configured = False
        self.create_subscription(State, f"{namespace}/state", self._state, 10)
        self.create_subscription(
            NavSatFix, f"{namespace}/global_position/global", self._fix,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, f"{namespace}/local_position/odom", self._odom,
            qos_profile_sensor_data)
        self.interval_client = self.create_client(
            MessageInterval, f"{namespace}/set_message_interval")
        # The same checks the tree makes: rcl_action_server_is_available
        # (goal, cancel and result services plus feedback and status
        # publishers) and service availability.
        self.action_clients = {
            name: ActionClient(self, interface_type(type_name), name)
            for name, type_name in sorted((actions or {}).items())}
        self.service_clients = {
            name: self.create_client(interface_type(type_name), name)
            for name, type_name in sorted((services or {}).items())}

    def _state(self, message):
        self.state = message

    def _fix(self, message):
        self.fix = message

    def _odom(self, message):
        self.odom = message

    def configure_intervals(self):
        if self.intervals_configured:
            return True
        if not (self.state and self.state.connected and
                self.interval_client.service_is_ready()):
            return False
        for message_id, rate_hz in MESSAGE_RATES_HZ.items():
            if not request_message_interval(
                    self, self.interval_client, message_id, rate_hz):
                self.get_logger().warning(
                    f"MAVLink interval request failed for message {message_id}; retrying")
                return False
        self.intervals_configured = True
        self.get_logger().info("Configured required MAVLink telemetry intervals")
        return True

    def missing_servers(self):
        missing = [name for name, client in self.action_clients.items()
                   if not client.server_is_ready()]
        missing += [name for name, client in self.service_clients.items()
                    if not client.service_is_ready()]
        return missing

    def waiting_for(self):
        """What is not ready yet; empty once the vehicle is ready to fly."""
        waiting = []
        if not (self.state and self.state.connected):
            waiting.append("MAVROS connection")
        elif not self.intervals_configured:
            waiting.append("MAVLink telemetry intervals")
        if self.require_gps and not (
                self.fix is not None and
                self.fix.status.status >= NavSatStatus.STATUS_FIX and
                math.isfinite(self.fix.latitude) and math.isfinite(self.fix.longitude)):
            waiting.append("GPS fix")
        if not (self.odom is not None and all(math.isfinite(value) for value in (
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                self.odom.pose.pose.position.z))):
            waiting.append("local odometry")
        return waiting + self.missing_servers()

    def ready(self):
        return not self.waiting_for()


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument(
        "--timeout", type=float, default=180.0,
        help="seconds to wait before failing; 0 waits indefinitely")
    parser.add_argument(
        "--require-gps", action=argparse.BooleanOptionalAction, default=True,
        help="wait for a GPS fix (--no-require-gps: local odometry is enough)")
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument(
        "--behavior-tree",
        help="Aerostack2 mission tree: also wait for every server its nodes call")
    parser.add_argument(
        "--behavior-namespace", default="/",
        help="namespace the tree runs in (behavior_trees.launch.py drone_id)")
    parser.add_argument("--report-seconds", type=float, default=30.0)
    parser.add_argument("--exec", dest="command", nargs=argparse.REMAINDER,
                        help="after readiness, replace this process with this command")
    tokens = list(sys.argv[1:] if argv is None else argv)
    # argparse otherwise interprets a child's `--` as its own separator.
    if '--exec' in tokens:
        index = tokens.index('--exec')
        args = parser.parse_args(tokens[:index])
        args.command = tokens[index + 1:]
        if not args.command:
            parser.error('--exec requires a command')
        return args
    return parser.parse_args(tokens)


def main(argv=None):
    args = parse_args(argv)

    actions, services = {}, {}
    if args.behavior_tree:
        with open(args.behavior_tree, encoding="utf-8") as tree:
            actions, services = behavior_tree_servers(tree.read(), args.behavior_namespace)

    # Our CLI (including the future child's launch arguments) is not ROS argv.
    rclpy.init(args=[])
    node = NavigationReadiness(
        args.namespace.rstrip("/"), actions, services, require_gps=args.require_gps)
    if actions or services:
        node.get_logger().info(
            "Mission tree needs " + ", ".join(sorted(actions) + sorted(services)))
    start = time.monotonic()
    deadline = start + args.timeout if args.timeout > 0 else math.inf
    next_report = start + args.report_seconds
    ready_since = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.configure_intervals()
            if node.ready():
                ready_since = ready_since or time.monotonic()
                if time.monotonic() - ready_since >= args.settle_seconds:
                    node.get_logger().info(
                        "Navigator ready: MAVROS connected with "
                        + ("GPS and " if args.require_gps else "")
                        + "local odometry"
                        + (f"; {len(actions) + len(services)} mission servers available"
                           f" after {time.monotonic() - start:.0f} s"
                           if actions or services else ""))
                    break
            else:
                ready_since = None
                if time.monotonic() >= next_report:
                    next_report = time.monotonic() + args.report_seconds
                    node.get_logger().info(
                        "Waiting for " + ", ".join(node.waiting_for()))
        else:
            raise RuntimeError(
                "Navigator not ready before timeout; still waiting for "
                + ", ".join(node.waiting_for()))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.command:
        os.execvp(args.command[0], args.command)


if __name__ == "__main__":
    main()

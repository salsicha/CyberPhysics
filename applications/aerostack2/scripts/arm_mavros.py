#!/usr/bin/env python3
"""Put a MAVROS-connected ArduPilot vehicle in GUIDED mode and arm it."""

import argparse
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node


class ArmSession(Node):
    def __init__(self, namespace, mode):
        super().__init__("navigator_arm_session")
        self.mode = mode
        self.state = None
        self.create_subscription(State, f"{namespace}/state", self._state, 10)
        self.arm_client = self.create_client(CommandBool, f"{namespace}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{namespace}/set_mode")

    def _state(self, msg):
        self.state = msg

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.state is not None and self.state.connected and
                    self.arm_client.service_is_ready() and self.mode_client.service_is_ready()):
                return True
        return False

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def activate(self, timeout):
        if not self.wait(timeout):
            raise RuntimeError("MAVROS did not connect before timeout")
        mode_request = SetMode.Request()
        mode_request.custom_mode = self.mode
        response = self.call(self.mode_client, mode_request)
        if response is None or not response.mode_sent:
            raise RuntimeError(f"ArduPilot rejected mode {self.mode}")
        arm_request = CommandBool.Request()
        arm_request.value = True
        response = self.call(self.arm_client, arm_request)
        if response is None or not response.success:
            raise RuntimeError("ArduPilot rejected arming")
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state and self.state.armed and self.state.mode == self.mode:
                self.get_logger().info(f"Navigator active: mode={self.mode}, armed=true")
                return
        raise RuntimeError("ArduPilot did not report the requested armed GUIDED state")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = ArmSession(args.namespace.rstrip("/"), args.mode)
    try:
        node.activate(args.timeout)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""Linux side of the UNO Q Sub demo (MCU side: unoq_sub.ino).

Subscribes to /micro_ros_arduino_subscriber (std_msgs/Int32), the topic of the
micro-ROS ``Sub`` demo, and calls the MCU's ``set_led`` RPC with the value
through the arduino-router Unix socket (msgpack-RPC: request
``[0, msgid, method, params]``, response ``[1, msgid, error, result]``).
Runs on the UNO Q in the ``unoq`` image (compositions/arduino_uno_q.yaml).
"""

from __future__ import annotations

import os
import socket

import msgpack
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32

ROUTER_SOCKET = "/var/run/arduino-router.sock"


class RouterClient:
    """Minimal blocking msgpack-RPC client for arduino-router."""

    def __init__(self, path: str, timeout_s: float) -> None:
        self.path = path
        self.timeout_s = timeout_s
        self.sock: socket.socket | None = None
        self.unpacker = msgpack.Unpacker(raw=False)
        self.msgid = 0

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def call(self, method: str, *params: object) -> object:
        """Call ``method`` and return its result; raises on error or timeout."""
        if self.sock is None:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout_s)
            self.sock.connect(self.path)
            self.unpacker = msgpack.Unpacker(raw=False)
        self.msgid = (self.msgid + 1) % (2**32)
        try:
            self.sock.sendall(msgpack.packb([0, self.msgid, method, list(params)]))
            while True:
                data = self.sock.recv(4096)
                if not data:
                    raise ConnectionError("arduino-router closed the connection")
                self.unpacker.feed(data)
                for message in self.unpacker:
                    if message[0] == 1 and message[1] == self.msgid:
                        if message[2] is not None:
                            raise RuntimeError(f"{method}: {message[2]}")
                        return message[3]
        except OSError:
            self.close()  # reconnect on the next call
            raise


class UnoQSubNode(Node):
    def __init__(self) -> None:
        super().__init__("unoq_sub")
        self.declare_parameter("topic", "/micro_ros_arduino_subscriber")
        self.declare_parameter(
            "router_socket", os.environ.get("UNOQ_ROUTER_SOCKET", ROUTER_SOCKET)
        )
        self.declare_parameter("rpc_timeout_s", 1.0)
        self.router = RouterClient(
            str(self.get_parameter("router_socket").value),
            float(self.get_parameter("rpc_timeout_s").value),
        )
        topic = str(self.get_parameter("topic").value)
        self.create_subscription(Int32, topic, self._led, 10)
        self.get_logger().info(f"{topic} -> set_led on the UNO Q MCU")

    def _led(self, msg: Int32) -> None:
        try:
            lit = self.router.call("set_led", int(msg.data))
        except (OSError, RuntimeError) as exc:
            self.get_logger().error(f"set_led({msg.data}) failed: {exc}")
            return
        self.get_logger().info(f"LED {'on' if lit else 'off'}")


def main() -> None:
    rclpy.init()
    node = UnoQSubNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.router.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

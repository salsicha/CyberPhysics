#!/usr/bin/env python3
"""Exercise the real MCU C code against native ROS over TCP and serial framing.

Serial tests use two pseudo terminals joined byte-for-byte. This tests the
actual Zenoh serial link and CDR implementation, not USB hardware/electronics.
"""

import argparse
import faulthandler
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import threading
import termios
import time
import tty


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(5)


def run(args):
    faulthandler.dump_traceback_later(120)
    # Import only after middleware environment is configured.
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    with socket.socket() as port_sock:
        port_sock.bind(("127.0.0.1", 0))
        port = port_sock.getsockname()[1]
    endpoint = f"tcp/127.0.0.1:{port}"
    os.environ["ZENOH_CONFIG_OVERRIDE"] = (
        f'mode="client";listen/endpoints=[];connect/endpoints=["{endpoint}"];scouting/multicast/enabled=false'
    )
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "-1"
    import rclpy
    from std_msgs.msg import Int32, String
    from rclpy.utilities import get_rmw_implementation_identifier

    handles, masters = [], []
    quit_relay = threading.Event()
    relay = None
    router = embedded = node = serial_router = None
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    router_log = (out / "router.log").open("w")
    embedded_log = (out / "embedded.log").open("w")
    try:
        listeners = [endpoint]
        locator = endpoint
        if args.transport == "serial":
            for _ in range(2):
                master, slave = pty.openpty()
                tty.setraw(slave)
                handles.extend([master, slave])
                masters.append(master)
            listeners.append(f"serial/{os.ttyname(handles[1])}#baudrate=115200")
            locator = f"serial/{os.ttyname(handles[3])}#baudrate=115200"

            def forward():
                while not quit_relay.is_set():
                    for source in select.select(masters, [], [], 0.05)[0]:
                        try:
                            data = os.read(source, 4096)
                            target = masters[1] if source == masters[0] else masters[0]
                            while data:
                                data = data[os.write(target, data) :]
                        except OSError:
                            if quit_relay.is_set():
                                return

            relay = threading.Thread(target=forward, daemon=True)
            relay.start()
        common_config = "scouting/multicast/enabled=false;transport/link/tx/lease=10000;transport/link/tx/keep_alive=2"
        router_env = {
            **os.environ,
            "ZENOH_CONFIG_OVERRIDE": f"listen/endpoints={json.dumps([endpoint] if args.routed else listeners)};connect/endpoints=[];{common_config}",
        }

        def start_serial_router():
            serial_env = {
                **os.environ,
                "ZENOH_CONFIG_OVERRIDE": f'listen/endpoints={json.dumps(listeners[1:])};connect/endpoints=["{endpoint}"];{common_config}',
            }
            return subprocess.Popen(
                ["/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd"],
                env=serial_env,
                stdout=router_log,
                stderr=subprocess.STDOUT,
            )

        def start_router():
            proc = subprocess.Popen(
                ["/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd"],
                env=router_env,
                stdout=router_log,
                stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(
                        "Router exited: " + (out / "router.log").read_text()[-1500:]
                    )
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                        return proc
                except OSError:
                    time.sleep(0.05)
            stop(proc)
            raise TimeoutError("Router TCP listener unavailable")

        router = start_router()
        if args.routed:
            if args.transport != "serial":
                raise ValueError("--routed requires --transport serial")
            serial_router = start_serial_router()
        embedded = subprocess.Popen(
            [args.binary, locator, str(args.domain)],
            stdout=embedded_log,
            stderr=subprocess.STDOUT,
        )
        rclpy.init()
        node = rclpy.create_node("embedded_zenoh_smoke")
        values, replies = [], []
        node.create_subscription(
            Int32, "/micro_ros_arduino_status", lambda m: values.append(m.data), 10
        )
        node.create_subscription(
            String, "/micro_ros_response", lambda m: replies.append(m.data), 10
        )
        led = node.create_publisher(Int32, "/micro_ros_arduino_subscriber", 10)
        name = node.create_publisher(String, "/micro_ros_name", 10)

        def exchange(value, text, timeout=20):
            values.clear()
            replies.clear()
            deadline = time.monotonic() + timeout
            next_send = 0
            while time.monotonic() < deadline:
                if embedded.poll() is not None:
                    raise RuntimeError(
                        "Embedded process exited: "
                        + (out / "embedded.log").read_text()[-1500:]
                    )
                if time.monotonic() >= next_send:
                    led.publish(Int32(data=value))
                    name.publish(String(data=text))
                    next_send = time.monotonic() + 0.2
                rclpy.spin_once(node, timeout_sec=0.05)
                if value in values and f"Hello {text}!" in replies:
                    return
            raise TimeoutError(
                f"{args.transport}: expected {value}/{text}, received {values}/{replies}"
            )

        tests = []
        for value, text in [
            (1, "Alice"),
            (0, "Bob"),
            (-1, "Zoë"),
            (-2147483648, "min"),
            (2147483647, "max"),
            (7, "x" * 120),
        ]:
            exchange(value, text)
            tests.append({"value": value, "name": text, "passed": True})
        print("Initial bidirectional traffic passed", flush=True)
        names = node.get_node_names()
        assert "micro_ros_platformio_node" in names, names
        # A router restart exercises session/endpoint redeclaration, not a new MCU process.
        stop(router)
        router = None
        # Parent-held PTY descriptors keep TIOCEXCL alive after the router dies.
        # A real serial device loses that lock when its last reader closes.
        for slave in handles[1::2]:
            fcntl.ioctl(slave, termios.TIOCNXCL)
        time.sleep(1)
        restart_started = time.monotonic()
        router = start_router()
        exchange(
            42, "after router restart", timeout=45 if args.transport == "serial" else 15
        )
        router_recovery_s = time.monotonic() - restart_started
        print(f"Router recovered in {router_recovery_s:.3f}s", flush=True)
        serial_router_recovery_s = None
        if args.routed:
            stop(serial_router)
            for slave in handles[1::2]:
                fcntl.ioctl(slave, termios.TIOCNXCL)
            restart_started = time.monotonic()
            serial_router = start_serial_router()
            exchange(44, "after serial router restart", timeout=45)
            serial_router_recovery_s = time.monotonic() - restart_started
        # A microcontroller restart must also reappear in the existing ROS graph.
        stop(embedded)
        embedded = subprocess.Popen(
            [args.binary, locator, str(args.domain)],
            stdout=embedded_log,
            stderr=subprocess.STDOUT,
        )
        exchange(43, "after MCU restart", timeout=30)
        # Malformed/oversized strings must be ignored without crashing the firmware.
        replies.clear()
        name.publish(String(data="x" * 121))
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert "Hello " + ("x" * 121) + "!" not in replies
        exchange(0, "still alive")
        # Both supported CDR byte orders must be decoded on the MCU.
        values.clear()
        led.publish(bytes([0, 0, 0, 0, 0x12, 0x34, 0x56, 0x78]))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and 0x12345678 not in values:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert 0x12345678 in values, values
        # Serialized malformed CDR must be rejected without updating the LED.
        values.clear()
        led.publish(bytes([0, 2, 0, 0, 1, 0, 0, 0]))  # unsupported CDR representation
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert 1 not in values, values
        exchange(0, "after malformed CDR")
        report = {
            "passed": True,
            "transport": args.transport,
            "rmw": get_rmw_implementation_identifier(),
            "domain": args.domain,
            "bidirectional_tests": tests,
            "router_restart": True,
            "mcu_restart": True,
            "oversized_string_rejected": True,
            "hardware_test": False,
            "router_recovery_s": router_recovery_s,
            "routed": args.routed,
            "serial_router_recovery_s": serial_router_recovery_s,
            "malformed_cdr_rejected": True,
            "big_endian_cdr": True,
        }
        (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop(embedded)
        stop(serial_router)
        stop(router)
        quit_relay.set()
        if relay:
            relay.join(1)
        for fd in handles:
            os.close(fd)
        router_log.close()
        embedded_log.close()
        faulthandler.cancel_dump_traceback_later()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="/opt/embedded-host/embedded_zenoh")
    parser.add_argument("--transport", choices=["tcp", "serial"], default="tcp")
    parser.add_argument("--domain", type=int, default=81)
    parser.add_argument("--routed", action="store_true")
    parser.add_argument("--output", default="/tmp/embedded-zenoh-smoke")
    run(parser.parse_args())

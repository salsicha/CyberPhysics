#!/usr/bin/env python3
"""Bridge between the PanTiltROS joint contract and the turret firmware.

Firmware: applications/arduino/scripts/turret_firmware. Two transports carry
the same messages (radians):

``serial`` (default; UNO R4 WiFi on USB): ASCII lines, newline terminated

  host -> firmware  ``SET pan_joint <pan> tilt_joint <tilt>``  move to position
  host -> firmware  ``STOP``                                    hold / stop motion
  firmware -> host  ``POS pan_joint <pan> tilt_joint <tilt>``  measured position
  firmware -> host  ``ERR <text>``                              error report

``uno_q_bridge`` (run on the Arduino UNO Q's Linux side): msgpack-RPC through
the arduino-router Unix socket to the board's MCU (Arduino_RouterBridge):
requests ``turret/set [pan, tilt]`` and ``turret/stop []``, and the
notifications ``turret/pos [pan, tilt]`` and ``turret/err [text]``, which the
router forwards here after ``$/register``. ``turret/get_position`` (also used
as a liveness probe) returns ``[pan, tilt]``, or ``[]`` without a fresh reading.

/turret/joint_states is published only from measured positions (``POS`` /
``turret/pos``); commands are never echoed back as joint states. Firmware that
does not report positions leaves the topic silent, and the tracker then relies
on /turret/actuator_ready, which is true only while commands are actually being
delivered: serial port open and settled, or the UNO Q MCU answering RPCs.
"""

from __future__ import annotations

import math
import os
import select
import socket
import time
from collections.abc import Callable
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import serial
except ImportError:  # pragma: no cover - depends on container build contents.
    serial = None

try:
    import msgpack
except ImportError:  # pragma: no cover - depends on container build contents.
    msgpack = None

JOINT_NAMES = ("pan_joint", "tilt_joint")
STOP_LINE = "STOP\n"
MAX_LINE_BYTES = 256

TRANSPORTS = ("serial", "uno_q_bridge")
ROUTER_SOCKET = "/var/run/arduino-router.sock"
# RouterBridge RPC methods provided by the UNO Q firmware
RPC_SET = "turret/set"
RPC_STOP = "turret/stop"
RPC_GET_POSITION = "turret/get_position"
# Notifications the firmware sends; the router routes them to whoever registered
RPC_POSITION = "turret/pos"
RPC_ERROR = "turret/err"
ROUTER_REGISTER = "$/register"
# arduino-router error codes (internal/msgpackrouter/errors.go)
ROUTER_METHOD_NOT_AVAILABLE = 2
ROUTER_ROUTE_ALREADY_EXISTS = 5
# Without any RPC answer for this long, probe the MCU with turret/get_position
PROBE_PERIOD_S = 1.0


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def format_command(pan: float, tilt: float) -> str:
    return f"SET pan_joint {pan:.6f} tilt_joint {tilt:.6f}\n"


def parse_position_line(line: str) -> Optional[tuple[float, float]]:
    """Parse ``POS pan_joint <rad> tilt_joint <rad>``; None if malformed."""
    tokens = line.split()
    if len(tokens) != 5 or tokens[0] != "POS":
        return None
    values = {}
    for name, raw in zip(tokens[1::2], tokens[2::2]):
        try:
            value = float(raw)
        except ValueError:
            return None
        if not math.isfinite(value):
            return None
        values[name] = value
    if set(values) != set(JOINT_NAMES):
        return None
    return values["pan_joint"], values["tilt_joint"]


def parse_position_params(params: object) -> tuple[float, float] | None:
    """Parse ``turret/pos`` / ``turret/get_position`` ``[pan, tilt]``; None if malformed."""
    if not isinstance(params, (list, tuple)) or len(params) != 2:
        return None
    values = []
    for value in params:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        if not math.isfinite(value):
            return None
        values.append(float(value))
    return values[0], values[1]


def format_rpc_error(error: object) -> str:
    """Text of a msgpack-RPC error: ``[code, message]`` or anything else."""
    if isinstance(error, (list, tuple)) and len(error) == 2:
        return f"{error[1]} (code {error[0]})"
    return str(error)


def rpc_error_code(error: object) -> int | None:
    if isinstance(error, (list, tuple)) and len(error) == 2 and isinstance(error[0], int):
        return error[0]
    return None


RpcCallback = Callable[[object, object], None]


class RouterBridgeLink:
    """Non-blocking msgpack-RPC client for the UNO Q's arduino-router socket.

    Messages are msgpack arrays: request ``[0, msgid, method, params]``,
    response ``[1, msgid, error, result]`` and notification
    ``[2, method, params]``. Requests and notifications for methods registered
    with ``$/register`` are routed to this client. Driven from the node's poll
    timer: request() only writes, poll() reads what has arrived, runs the
    response callbacks, and fails requests unanswered after ``timeout_s``.
    """

    def __init__(self, path: str, timeout_s: float) -> None:
        self.path = path
        self.timeout_s = timeout_s
        self.sock: socket.socket | None = None
        self.unpacker = None
        self.next_msgid = 0
        # msgid -> (method, deadline, callback(error, result))
        self.pending: dict[int, tuple[str, float, RpcCallback]] = {}

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def connect(self) -> None:
        if msgpack is None:
            raise RuntimeError("python3-msgpack is not installed in this image")
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.connect(self.path)
        except OSError:
            sock.close()
            raise
        self.sock = sock
        self.unpacker = msgpack.Unpacker(raw=False)
        self.pending = {}

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None
        pending, self.pending = self.pending, {}
        for _method, _deadline, callback in pending.values():
            callback("connection to arduino-router closed", None)

    def _send(self, message: list) -> None:
        if self.sock is None:
            raise ConnectionError("not connected to arduino-router")
        try:
            self.sock.sendall(msgpack.packb(message))
        except OSError:
            self.close()
            raise

    def request(self, method: str, params: tuple | list, callback: RpcCallback) -> None:
        """Send a request; callback(error, result) runs from a later poll()."""
        self.next_msgid = (self.next_msgid + 1) % (2**32)
        msgid = self.next_msgid
        self._send([0, msgid, method, list(params)])
        self.pending[msgid] = (method, time.monotonic() + self.timeout_s, callback)

    def notify(self, method: str, params: tuple | list) -> None:
        self._send([2, method, list(params)])

    def respond(self, msgid: int, error: object, result: object) -> None:
        self._send([1, msgid, error, result])

    def poll(self) -> list[tuple[int, str, list, int | None]]:
        """Process received data; return incoming (type, method, params, msgid).

        type is 0 for requests (answer with respond()) and 2 for notifications.
        """
        incoming: list[tuple[int, str, list, int | None]] = []
        while self.sock is not None:
            try:
                readable, _, _ = select.select([self.sock], [], [], 0)
                if not readable:
                    break
                data = self.sock.recv(4096)
            except OSError:
                self.close()
                raise
            if not data:
                self.close()
                raise ConnectionError("arduino-router closed the connection")
            self.unpacker.feed(data)
            for message in self.unpacker:
                self._dispatch(message, incoming)
        now = time.monotonic()
        for msgid, (method, deadline, callback) in list(self.pending.items()):
            if now >= deadline and self.pending.pop(msgid, None) is not None:
                callback(f"no answer to {method} within {self.timeout_s:g} s", None)
        return incoming

    def _dispatch(self, message: object, incoming: list) -> None:
        if not isinstance(message, (list, tuple)) or not message:
            return
        kind = message[0]
        if kind == 1 and len(message) == 4:
            entry = self.pending.pop(message[1], None)
            if entry is not None:
                entry[2](message[2], message[3])
        elif kind == 0 and len(message) == 4:
            params = message[3] if isinstance(message[3], (list, tuple)) else []
            incoming.append((0, str(message[2]), list(params), message[1]))
        elif kind == 2 and len(message) == 3:
            params = message[2] if isinstance(message[2], (list, tuple)) else []
            incoming.append((2, str(message[1]), list(params), None))


class TurretArduinoBridge(Node):
    # Transport state defaults; __init__ sets them from the parameters.
    transport = "serial"
    router = None
    mcu_ready = False
    registered: frozenset[str] | set[str] = frozenset()

    def __init__(self) -> None:
        super().__init__("turret_arduino_bridge")
        self.declare_parameter("joint_command_topic", "/turret/joint_commands")
        self.declare_parameter("joint_state_topic", "/turret/joint_states")
        self.declare_parameter("diagnostics_topic", "/turret/diagnostics")
        self.declare_parameter("emergency_stop_topic", "/turret/emergency_stop")
        self.declare_parameter("actuator_ready_topic", "/turret/actuator_ready")
        self.declare_parameter(
            "serial_port", os.environ.get("TURRET_ARDUINO_PORT", "/dev/ttyACM0")
        )
        self.declare_parameter(
            "baud_rate", int(os.environ.get("TURRET_ARDUINO_BAUD", "115200"))
        )
        self.declare_parameter(
            "dry_run", env_bool("TURRET_ARDUINO_DRY_RUN", False)
        )
        # serial: line protocol on serial_port; uno_q_bridge: RPC to the UNO Q
        # MCU through the arduino-router socket (run on the UNO Q itself)
        self.declare_parameter(
            "transport", os.environ.get("TURRET_ARDUINO_TRANSPORT", "serial")
        )
        self.declare_parameter(
            "router_socket", os.environ.get("TURRET_ROUTER_SOCKET", ROUTER_SOCKET)
        )
        # uno_q_bridge: an RPC unanswered for this long marks the MCU not ready
        self.declare_parameter("rpc_timeout_s", 0.5)
        # Nothing is written until an explicit emergency-stop release (False).
        self.declare_parameter("require_estop_release", True)
        self.declare_parameter("diagnostic_rate_hz", 1.0)
        self.declare_parameter("poll_rate_hz", 50.0)
        self.declare_parameter("ready_rate_hz", 5.0)
        # Defaults match the shared PanTiltROS turret contract: tilt is restricted
        # to keep the payload clear of the base.
        self.declare_parameter("pan_min_rad", -1.5708)
        self.declare_parameter("pan_max_rad", 1.5708)
        self.declare_parameter("tilt_min_rad", -0.65)
        self.declare_parameter("tilt_max_rad", 0.75)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.transport = str(self.get_parameter("transport").value).strip().lower()
        if self.transport not in TRANSPORTS:
            raise ValueError(
                f"transport must be one of {', '.join(TRANSPORTS)}, got {self.transport!r}"
            )
        self.router_socket = str(self.get_parameter("router_socket").value)
        self.rpc_timeout_s = max(0.05, float(self.get_parameter("rpc_timeout_s").value))
        self.router: RouterBridgeLink | None = None
        self.mcu_ready = False
        self.registered = set()  # notifications the router now routes to us
        self.last_probe_time = float("-inf")
        self.probe_pending = False
        self.pan_min_rad = float(self.get_parameter("pan_min_rad").value)
        self.pan_max_rad = float(self.get_parameter("pan_max_rad").value)
        self.tilt_min_rad = float(self.get_parameter("tilt_min_rad").value)
        self.tilt_max_rad = float(self.get_parameter("tilt_max_rad").value)
        self.serial_handle = None
        self.last_error = ""
        self.last_write_time: Optional[float] = None
        self.last_feedback_time: Optional[float] = None
        self.connect_settle_s = 2.0
        self.reconnect_interval_s = 2.0
        self.serial_ready_time = 0.0
        self.last_connect_attempt = float("-inf")
        self.rx_buffer = b""
        self.estop_engaged = bool(self.get_parameter("require_estop_release").value)
        self.stop_sent = False
        self.dropped_commands = 0
        self.last_ready: Optional[bool] = None

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("actuator_ready_topic").value), latched_qos
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_command_topic").value),
            self._joint_command,
            10,
        )
        # Latched for transient-local publishers, volatile for default-QoS ones.
        for qos in (latched_qos, 10):
            self.create_subscription(
                Bool,
                str(self.get_parameter("emergency_stop_topic").value),
                self._emergency_stop,
                qos,
            )

        rate_hz = max(0.2, float(self.get_parameter("diagnostic_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_diagnostics)
        poll_hz = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self.create_timer(1.0 / poll_hz, self._poll)
        ready_hz = max(1.0, float(self.get_parameter("ready_rate_hz").value))
        self.create_timer(1.0 / ready_hz, self._publish_ready)
        self._connect()
        if self.transport == "uno_q_bridge":
            self.get_logger().info(
                f"Turret Arduino bridge ready on the UNO Q router socket {self.router_socket}"
            )
        else:
            self.get_logger().info(
                "Turret Arduino bridge ready on "
                f"{self.serial_port} at {self.baud_rate} baud"
            )

    def _connect(self) -> None:
        if self.transport == "uno_q_bridge":
            self._connect_router()
        else:
            self._connect_serial()

    def _connect_serial(self) -> None:
        if self.dry_run:
            self.last_error = "dry_run enabled"
            return
        if serial is None:
            self.last_error = "pyserial is not installed in this image"
            return
        if self.serial_handle and self.serial_handle.is_open:
            return
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval_s:
            return
        self.last_connect_attempt = now
        try:
            self.serial_handle = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.05,
                write_timeout=0.1,
            )
            # Non-blocking settle: skip writes until the Arduino has rebooted.
            self.serial_ready_time = now + self.connect_settle_s
            self.rx_buffer = b""
            self.stop_sent = False
            self.last_error = ""
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.serial_handle = None
            self.last_error = str(exc)

    def _connect_router(self) -> None:
        if self.dry_run:
            self.last_error = "dry_run enabled"
            return
        if self._connected():
            return
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval_s:
            return
        self.last_connect_attempt = now
        link = RouterBridgeLink(self.router_socket, self.rpc_timeout_s)
        try:
            link.connect()
        except (OSError, RuntimeError) as exc:
            self.last_error = f"arduino-router socket {self.router_socket}: {exc}"
            return
        self.router = link
        self.mcu_ready = False
        self.registered = set()
        self.stop_sent = False
        self.probe_pending = False
        self.last_probe_time = float("-inf")
        self.last_error = ""
        # Receive the firmware's position and error notifications
        for method in (RPC_POSITION, RPC_ERROR):
            self._rpc(
                ROUTER_REGISTER,
                (method,),
                lambda error, _result, method=method: self._registered(method, error),
            )
        self._probe()

    def _registered(self, method: str, error: object) -> None:
        if error is None:
            self.registered.add(method)
            return
        if rpc_error_code(error) == ROUTER_ROUTE_ALREADY_EXISTS:
            self.last_error = (
                f"another arduino-router client already receives {method}; "
                "is a second turret bridge running?"
            )
        else:
            self.last_error = f"cannot register {method}: {format_rpc_error(error)}"

    def _rpc(self, method: str, params: tuple, callback: RpcCallback | None = None) -> bool:
        """Send an RPC request to the UNO Q; False if it could not be written."""
        if not self._connected():
            return False
        try:
            self.router.request(
                method,
                params,
                callback or (lambda error, result: self._rpc_done(method, error, result)),
            )
        except OSError as exc:
            self.last_error = f"arduino-router: {exc}"
            self._close_router()
            return False
        return True

    def _rpc_done(self, method: str, error: object, result: object) -> None:
        if error is not None:
            if self.mcu_ready:
                self.get_logger().warning(f"UNO Q MCU not ready: {method}: {error}")
            self.mcu_ready = False
            if rpc_error_code(error) == ROUTER_METHOD_NOT_AVAILABLE:
                self.last_error = (
                    f"{method} not available: is turret_firmware running on the UNO Q MCU?"
                )
            else:
                self.last_error = f"{method}: {format_rpc_error(error)}"
            return
        if not self.mcu_ready:
            self.get_logger().info("UNO Q MCU answering turret RPCs")
            # Possibly a restarted MCU: repeat STOP while the e-stop is engaged
            self.stop_sent = False
            if {RPC_POSITION, RPC_ERROR} <= self.registered:
                self.last_error = ""  # keep a registration error visible
        self.mcu_ready = True
        if method == RPC_SET and result is False:
            self.last_error = "firmware rejected SET"

    def _probe(self) -> None:
        def done(error: object, result: object) -> None:
            self.probe_pending = False
            self._rpc_done(RPC_GET_POSITION, error, result)

        self.last_probe_time = time.monotonic()
        self.probe_pending = self._rpc(RPC_GET_POSITION, (), done)

    def _connected(self) -> bool:
        if self.transport == "uno_q_bridge":
            return bool(self.router and self.router.connected)
        return bool(self.serial_handle and self.serial_handle.is_open)

    def _settling(self) -> bool:
        if self.transport == "uno_q_bridge":
            # Connected to the router, but the MCU has not answered (yet), or
            # its notifications are not routed here (another bridge owns them)
            return self._connected() and not (
                self.mcu_ready and {RPC_POSITION, RPC_ERROR} <= self.registered
            )
        return self._connected() and time.monotonic() < self.serial_ready_time

    def actuator_ready(self) -> bool:
        """True only while SET commands would actually be written."""
        return (
            not self.dry_run
            and not self.estop_engaged
            and self._connected()
            and not self._settling()
        )

    def _write(self, line: str) -> bool:
        try:
            self.serial_handle.write(line.encode("ascii"))
            self.serial_handle.flush()
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.last_error = str(exc)
            self._close_serial()
            return False
        self.last_error = ""
        return True

    def _send_set(self, pan: float, tilt: float) -> bool:
        if self.transport == "uno_q_bridge":
            return self._send_rpc(RPC_SET, (pan, tilt))
        return self._write(format_command(pan, tilt))

    def _send_stop(self) -> bool:
        if self.transport == "uno_q_bridge":
            return self._send_rpc(RPC_STOP, ())
        return self._write(STOP_LINE)

    def _send_rpc(self, method: str, params: tuple) -> bool:
        # Like a successful serial write, a delivered command clears last_error
        sent = self._rpc(method, params)
        if sent:
            self.last_error = ""
        return sent

    def _joint_command(self, msg: JointState) -> None:
        pan = self._joint_position(msg, "pan_joint")
        tilt = self._joint_position(msg, "tilt_joint")
        if pan is None or tilt is None:
            self.last_error = "joint command must include pan_joint and tilt_joint"
            return
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            self.last_error = f"rejected non-finite joint command ({pan}, {tilt})"
            self.get_logger().warning(self.last_error, throttle_duration_sec=5.0)
            return
        if not self.actuator_ready():
            # Never queue: a command held through an outage or e-stop would be
            # executed as a full-speed jump when the link comes back.
            self.dropped_commands += 1
            return
        pan = min(max(pan, self.pan_min_rad), self.pan_max_rad)
        tilt = min(max(tilt, self.tilt_min_rad), self.tilt_max_rad)
        if self._send_set(pan, tilt):
            self.last_write_time = time.monotonic()

    def _joint_position(self, msg: JointState, name: str) -> Optional[float]:
        if name in msg.name:
            index = msg.name.index(name)
            if index < len(msg.position):
                return float(msg.position[index])
        return None

    def _emergency_stop(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        if engaged != self.estop_engaged:
            self.get_logger().warning(
                "emergency stop " + ("engaged" if engaged else "released")
            )
        self.estop_engaged = engaged
        if engaged:
            self.stop_sent = False
            self._send_stop_if_needed()
        self._publish_ready()

    def _send_stop_if_needed(self) -> None:
        if (
            self.estop_engaged
            and not self.stop_sent
            and not self.dry_run
            and self._connected()
            and not self._settling()
        ):
            self.stop_sent = self._send_stop()

    def _poll(self) -> None:
        if self.dry_run:
            return
        if self.transport == "uno_q_bridge":
            self._poll_router()
            return
        self._connect_serial()
        if not self._connected():
            return
        try:
            waiting = self.serial_handle.in_waiting
            data = self.serial_handle.read(waiting) if waiting else b""
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.last_error = str(exc)
            self._close_serial()
            return
        self._handle_serial_data(data)
        self._send_stop_if_needed()
        if self.actuator_ready() != self.last_ready:
            self._publish_ready()

    def _poll_router(self) -> None:
        self._connect_router()
        if not self._connected():
            return
        try:
            incoming = self.router.poll()
        except OSError as exc:
            self.last_error = f"arduino-router: {exc}"
            self._close_router()
            return
        for kind, method, params, msgid in incoming:
            self._handle_router_message(method, params)
            if kind == 0 and self._connected():
                # The firmware used call() instead of notify(): acknowledge it
                try:
                    self.router.respond(msgid, None, True)
                except OSError as exc:
                    self.last_error = f"arduino-router: {exc}"
                    self._close_router()
                    return
        if not self._connected():
            return
        if (
            not self.probe_pending
            and time.monotonic() - self.last_probe_time >= PROBE_PERIOD_S
        ):
            self._probe()
        self._send_stop_if_needed()
        if self.actuator_ready() != self.last_ready:
            self._publish_ready()

    def _handle_router_message(self, method: str, params: list) -> None:
        if method == RPC_POSITION:
            position = parse_position_params(params)
            if position is not None:
                self._publish_joint_state(*position)
        elif method == RPC_ERROR:
            text = params[0] if params else ""
            self.last_error = f"firmware: {str(text).strip()}"

    def _handle_serial_data(self, data: bytes) -> None:
        self.rx_buffer += data
        *lines, self.rx_buffer = self.rx_buffer.split(b"\n")
        if len(self.rx_buffer) > MAX_LINE_BYTES:
            self.rx_buffer = b""
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            position = parse_position_line(line)
            if position is not None:
                self._publish_joint_state(*position)
            elif line.startswith("ERR"):
                self.last_error = f"firmware: {line[3:].strip()}"

    def _publish_joint_state(self, pan: float, tilt: float) -> None:
        self.last_feedback_time = time.monotonic()
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(JOINT_NAMES)
        state.position = [pan, tilt]
        self.joint_state_pub.publish(state)

    def _publish_ready(self) -> None:
        ready = self.actuator_ready()
        self.last_ready = ready
        self.ready_pub.publish(Bool(data=ready))

    def _age(self, stamp: Optional[float]) -> str:
        return "never" if stamp is None else f"{time.monotonic() - stamp:.2f}"

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "turret_arduino_bridge"
        uno_q = self.transport == "uno_q_bridge"
        status.hardware_id = self.router_socket if uno_q else self.serial_port
        connected = self._connected()
        feedback = (
            self.last_feedback_time is not None
            and time.monotonic() - self.last_feedback_time < 1.0
        )
        if self.dry_run:
            status.level = DiagnosticStatus.WARN
            status.message = "dry run: commands are not sent"
        elif not connected or self.last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error or (
                "arduino-router socket disconnected" if uno_q else "serial disconnected"
            )
        elif self.estop_engaged:
            status.level = DiagnosticStatus.WARN
            status.message = "emergency stop engaged (publish false to release)"
        elif self._settling():
            status.level = DiagnosticStatus.WARN
            status.message = (
                "waiting for the UNO Q MCU to answer turret RPCs"
                if uno_q
                else "waiting for the Arduino to reset"
            )
        else:
            status.level = DiagnosticStatus.OK
            link = "UNO Q MCU connected" if uno_q else "serial connected"
            status.message = (
                link if feedback else f"{link}, no position feedback from firmware"
            )
        if uno_q:
            status.values = [KeyValue(key="router_socket", value=self.router_socket)]
        else:
            status.values = [
                KeyValue(key="port", value=self.serial_port),
                KeyValue(key="baud_rate", value=str(self.baud_rate)),
            ]
        status.values += [
            KeyValue(key="transport", value=self.transport),
            KeyValue(key="connected", value=str(connected).lower()),
            KeyValue(key="ready", value=str(self.actuator_ready()).lower()),
            KeyValue(key="emergency_stop", value=str(self.estop_engaged).lower()),
            KeyValue(key="position_feedback", value=str(feedback).lower()),
            KeyValue(key="last_feedback_age_s", value=self._age(self.last_feedback_time)),
            KeyValue(key="last_write_age_s", value=self._age(self.last_write_time)),
            KeyValue(key="dropped_commands", value=str(self.dropped_commands)),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)

    def _close_serial(self) -> None:
        if self.serial_handle:
            try:
                self.serial_handle.close()
            finally:
                self.serial_handle = None
        self.stop_sent = False

    def _close_router(self) -> None:
        router, self.router = self.router, None
        error = self.last_error
        # Not ready before failing the pending requests (their callbacks run
        # here, also at shutdown, when logging is no longer possible)
        self.mcu_ready = False
        if router is not None:
            router.close()
        self.last_error = error or self.last_error
        self.registered = set()
        self.probe_pending = False
        self.stop_sent = False

    def destroy_node(self) -> bool:
        if not self.dry_run and self._connected():
            self._send_stop()
        if self.transport == "uno_q_bridge":
            self._close_router()
        else:
            self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TurretArduinoBridge()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

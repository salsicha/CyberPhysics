#!/usr/bin/env python3
"""Bridge between the PanTiltROS joint contract and the turret firmware.

Firmware: applications/arduino/scripts/turret_firmware. Two transports carry
the same messages (radians) to it, and a third drives the servos itself:

``serial`` (default; UNO R4 WiFi on USB): ASCII lines, newline terminated

  host -> firmware  ``SET pan_joint <pan> tilt_joint <tilt>
                    [actuator_joint <m> trigger_joint <rad>]``
  host -> firmware  ``STOP``    hold, trigger to 0, rack stopped
  firmware -> host  ``POS pan_joint <pan> tilt_joint <tilt>
                    [actuator_joint <m> trigger_joint <rad>]``
  firmware -> host  ``ERR <text>``          error report

``uno_q_bridge`` (run on the Arduino UNO Q's Linux side): msgpack-RPC through
the arduino-router Unix socket to the board's MCU (Arduino_RouterBridge):
requests ``turret/set [pan, tilt]``, ``turret/set_all [pan, tilt, actuator,
trigger]`` and ``turret/stop []``, and the notifications ``turret/pos [pan,
tilt]`` (or ``[pan, tilt, actuator, trigger]``) and ``turret/err [text]``,
which the router forwards here after ``$/register``. ``turret/get_position``
(also used as a liveness probe) returns the same list, or ``[]`` without a
fresh reading.

``sts`` (no Arduino; e.g. a Jetson's header UART or a USB bus servo adapter):
the Feetech STS protocol straight to the ST3215 servos on serial_port at
1 Mbps through a half-duplex adapter. StsController is a port of the
firmware's servo code (src/sts_servo.*, Controller in src/turret_core.*), so
SET and STOP have the same effect (limits, torque on at the first SET, hold
after 500 ms without SET, 25 Hz feedback only when pan and tilt answer, servo
errors at most once per second) and its POS and ERR reports feed the same
topics and diagnostics. The watchdog then runs in this process: if the bridge
dies the servos keep their last goal.

With ``actuator_fitted`` (``TURRET_ACTUATOR_FITTED``) the foam dart launcher
(systems/racecar_turret/interfaces.md, "Foam dart launcher": its priming
rack, a third ST3215, ID 14, ``actuator_joint`` in metres, 0 forward; its
trigger, ID 15, ``trigger_joint`` in radians, 0 safe) travels with the SETs
of joint commands that carry ``actuator_joint``: both clamped to their
limits, the trigger at 0 when the command does not carry it. A command
without ``actuator_joint`` sends pan and tilt only, which returns the
trigger to 0 and stops the rack where it is. STOP, the emergency stop and
the 500 ms watchdog do the same; pan and tilt keep their hold behaviour. The rack servo's torque is limited (``actuator_torque_
limit``, 25 %) at every torque enable. The launcher's measured positions
join /turret/joint_states when both servos answered. Without a launcher
fitted, commands' launcher joints are ignored and joint states carry pan and
tilt only.

With ``pan_tilt_fitted`` false (``TURRET_PAN_TILT_FITTED``; sts transport,
launcher fitted) the bus carries the launcher alone, as on the aerodrone,
whose launcher points straight down with no pan or tilt
(systems/aerodrone/tracking.md, "Foam dart launcher"): nothing is ever
written to or read from the pan and tilt servo IDs, joint commands need not
carry pan_joint and tilt_joint (they are ignored), and joint states carry
the launcher's two joints when both its servos answered. The safety
behaviour is the same: a command without actuator_joint, STOP, the
emergency stop and the watchdog return the trigger to 0 and stop the rack.

/turret/joint_states is published only from measured positions (``POS`` /
``turret/pos`` / servo replies); commands are never echoed back as joint
states. Firmware that does not report positions leaves the topic silent, and
the tracker then relies on /turret/actuator_ready, which is true only while
commands are actually being delivered: serial port open and settled, or the
UNO Q MCU answering RPCs.
"""

from __future__ import annotations

import math
import os
import select
import socket
import sys
import time
from collections.abc import Callable, Sequence
from typing import NamedTuple, Optional

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
ACTUATOR_JOINT = "actuator_joint"  # the launcher's priming rack, metres
TRIGGER_JOINT = "trigger_joint"  # the launcher's trigger, radians
STOP_LINE = "STOP\n"
MAX_LINE_BYTES = 256

TRANSPORTS = ("serial", "uno_q_bridge", "sts")
ROUTER_SOCKET = "/var/run/arduino-router.sock"


class RouterUnreachable(RuntimeError):
    """The arduino-router socket stayed unreachable for router_lost_exit_s."""
# RouterBridge RPC methods provided by the UNO Q firmware
RPC_SET = "turret/set"
RPC_SET_ALL = "turret/set_all"  # pan, tilt and the launcher's rack and trigger
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


def format_command(
    pan: float, tilt: float, launcher: tuple[float, float] | None = None
) -> str:
    """A SET line; launcher is (rack metres, trigger radians) or None."""
    line = f"SET pan_joint {pan:.6f} tilt_joint {tilt:.6f}"
    if launcher is not None:
        actuator, trigger = launcher
        line += f" {ACTUATOR_JOINT} {actuator:.6f} {TRIGGER_JOINT} {trigger:.6f}"
    return line + "\n"


def parse_position_line(line: str) -> Optional[tuple[float, ...]]:
    """Parse ``POS pan_joint <rad> tilt_joint <rad> [actuator_joint <m>
    trigger_joint <rad>]``: (pan, tilt) or (pan, tilt, actuator, trigger);
    None if malformed."""
    tokens = line.split()
    if len(tokens) not in (5, 9) or tokens[0] != "POS":
        return None
    values = {}
    for name, raw in zip(tokens[1::2], tokens[2::2]):
        try:
            value = float(raw)
        except ValueError:
            return None
        if not math.isfinite(value) or name in values:
            return None
        values[name] = value
    launcher = {ACTUATOR_JOINT, TRIGGER_JOINT}
    if set(values) not in (set(JOINT_NAMES), set(JOINT_NAMES) | launcher):
        return None
    if ACTUATOR_JOINT in values:
        return (values["pan_joint"], values["tilt_joint"], values[ACTUATOR_JOINT],
                values[TRIGGER_JOINT])
    return values["pan_joint"], values["tilt_joint"]


def parse_position_params(params: object) -> tuple[float, ...] | None:
    """Parse ``turret/pos`` / ``turret/get_position`` ``[pan, tilt]`` or
    ``[pan, tilt, actuator, trigger]``; None if malformed."""
    if not isinstance(params, (list, tuple)) or len(params) not in (2, 4):
        return None
    values = []
    for value in params:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        if not math.isfinite(value):
            return None
        values.append(float(value))
    return tuple(values)


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


# ---- Feetech STS servos on this host's serial port (transport sts) ----
# Instruction packet: 0xFF 0xFF ID LEN INSTR PARAM... CHECKSUM
# Status packet:      0xFF 0xFF ID LEN ERROR PARAM... CHECKSUM
# LEN counts INSTR/ERROR, the parameters and the checksum (parameters + 2);
# CHECKSUM = ~(ID + LEN + INSTR/ERROR + parameters), low byte. STS servos
# store 16-bit registers little endian. Same packets as the firmware's
# src/sts_servo.cpp and Feetech's SCServo_Linux (tests/test_turret_bridge_sts.py).

STS_BROADCAST_ID = 0xFE
STS_INST_PING = 0x01
STS_INST_READ = 0x02
STS_INST_WRITE = 0x03
STS_INST_SYNC_WRITE = 0x83
# Memory table (SMS_STS.h in Feetech's SCServo library)
STS_REG_TORQUE_ENABLE = 40  # 0 off, 1 on; 128 recalibrates the midpoint: never written here
STS_REG_ACCELERATION = 41  # 41 acc, 42-43 goal position, 44-45 goal time, 46-47 goal speed
STS_REG_TORQUE_LIMIT = 48  # 48-49, 0.1 % units; reset from EEPROM's max torque at power-up
STS_REG_PRESENT_POSITION = 56
STS_STEPS_PER_TURN = 4096
STS_MAX_STEP = STS_STEPS_PER_TURN - 1
STS_STEPS_PER_RAD = STS_STEPS_PER_TURN / (2.0 * math.pi)
# Largest packet built or parsed (a four-servo sync write is 40 bytes)
STS_MAX_PACKET = 64
STS_MAX_STATUS_PARAMS = STS_MAX_PACKET - 6
# Status packet error bits, as the firmware names them
STS_FAULTS = ((0x01, "voltage"), (0x02, "angle-sensor"), (0x04, "overheat"),
              (0x08, "overcurrent"), (0x20, "overload"))
# Transaction results (the firmware's resultName())
STS_OK = "ok"
STS_TIMEOUT = "timeout"
STS_BAD_REPLY = "bad reply"
# Read timeout of the port; the bus waits for a reply in steps of this
STS_READ_TIMEOUT_S = 0.001


def sts_checksum(body: bytes) -> int:
    """~(sum of bytes) over ID..last parameter, low byte."""
    return ~sum(body) & 0xFF


def sts_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    if len(params) + 6 > STS_MAX_PACKET:
        raise ValueError(f"STS packet with {len(params)} parameters is too long")
    body = bytes((servo_id, len(params) + 2, instruction)) + bytes(params)
    return b"\xff\xff" + body + bytes((sts_checksum(body),))


def sts_ping(servo_id: int) -> bytes:
    return sts_packet(servo_id, STS_INST_PING)


def sts_read(servo_id: int, address: int, count: int) -> bytes:
    return sts_packet(servo_id, STS_INST_READ, bytes((address, count)))


def sts_write(servo_id: int, address: int, data: bytes) -> bytes:
    return sts_packet(servo_id, STS_INST_WRITE, bytes((address,)) + bytes(data))


def sts_sync_write_position(
    ids: Sequence[int], steps: Sequence[int], speed: int, acceleration: int
) -> bytes:
    """SYNC WRITE (broadcast, no reply) of acceleration, goal position, goal
    time 0 and goal speed (registers 41-47), as SMS_STS::SyncWritePosEx."""
    if not ids or len(ids) != len(steps):
        raise ValueError("sync write needs one position per servo id")
    params = bytearray((STS_REG_ACCELERATION, 7))
    for servo_id, step in zip(ids, steps):
        position = sts_encode_position(step)
        params += bytes((
            servo_id, acceleration, position & 0xFF, position >> 8,
            0, 0,  # goal time: unused in position mode
            speed & 0xFF, speed >> 8,
        ))
    return sts_packet(STS_BROADCAST_ID, STS_INST_SYNC_WRITE, bytes(params))


def sts_encode_position(steps: int) -> int:
    """Position registers use bit 15 as the sign (sign and magnitude)."""
    if steps < 0:
        return min(-steps, 0x7FFF) | 0x8000
    return min(steps, 0x7FFF)


def sts_decode_position(raw: int) -> int:
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


class StsStatusParser:
    """Incremental parser for status packets (the firmware's StatusParser).

    Leading garbage and extra 0xFF header bytes are skipped; packets with a
    bad length or checksum are dropped and counted in bad_packets.
    """

    HEADER1, HEADER2, ID, LENGTH, BODY = range(5)

    def __init__(self) -> None:
        self.bad_packets = 0
        self.reset()

    def reset(self) -> None:
        self.state = self.HEADER1
        self.id = 0
        self.length = 0
        self.error = 0
        self.params = bytearray()
        self.body_received = 0
        self.sum = 0

    def feed(self, byte: int) -> bool:
        """True when byte completes a valid packet (id, error, params)."""
        if self.state == self.HEADER1:
            if byte == 0xFF:
                self.state = self.HEADER2
            return False
        if self.state == self.HEADER2:
            self.state = self.ID if byte == 0xFF else self.HEADER1
            return False
        if self.state == self.ID:
            if byte == 0xFF:
                return False  # extra header byte; IDs are 0-253
            self.id = byte
            self.sum = byte
            self.state = self.LENGTH
            return False
        if self.state == self.LENGTH:
            if byte < 2 or byte > STS_MAX_STATUS_PARAMS + 2:
                self.bad_packets += 1
                self.state = self.HEADER2 if byte == 0xFF else self.HEADER1
                return False
            self.length = byte
            self.sum = (self.sum + byte) & 0xFF
            self.body_received = 0
            self.params = bytearray()
            self.state = self.BODY
            return False
        self.body_received += 1
        if self.body_received == self.length:  # checksum byte
            self.state = self.HEADER1
            if ~self.sum & 0xFF != byte:
                self.bad_packets += 1
                return False
            return True
        self.sum = (self.sum + byte) & 0xFF
        if self.body_received == 1:
            self.error = byte
        else:
            self.params.append(byte)
        return False

    def equals(self, packet: bytes) -> bool:
        """True if the completed packet is byte for byte ``packet`` (a
        half-duplex echo of what was just sent)."""
        return (
            len(packet) == len(self.params) + 6
            and packet[2] == self.id
            and packet[3] == self.length
            and packet[4] == self.error
            and packet[5:-1] == self.params
        )


class StsLinkError(OSError):
    """The servo bus's serial port failed (unplugged, closed)."""


class StsBus:
    """STS transactions on a half-duplex servo bus behind a pyserial port.

    As the firmware's sts::Bus: drain the input, write the instruction, then
    read until the addressed servo's status packet arrives or reply_timeout_s
    passes, skipping our own packet when the adapter echoes TX back to RX.
    """

    def __init__(self, port, reply_timeout_s: float) -> None:
        self.port = port
        self.reply_timeout_s = reply_timeout_s
        self.last_status = 0  # error byte of the last status packet (0 = no fault)
        self.bad_packets = 0

    def _send(self, packet: bytes) -> None:
        try:
            self.port.reset_input_buffer()  # late replies and echoes
            self.port.write(packet)
            self.port.flush()  # returns once the last byte has left the UART
        except Exception as exc:  # SerialException, termios.error, ...
            raise StsLinkError(f"servo bus: {exc}") from exc

    def _receive(self) -> bytes:
        try:
            return self.port.read(max(1, self.port.in_waiting))
        except Exception as exc:  # SerialException, termios.error, ...
            raise StsLinkError(f"servo bus: {exc}") from exc

    def _transact(self, packet: bytes, servo_id: int) -> tuple[str, StsStatusParser]:
        self._send(packet)
        reply = StsStatusParser()
        # perf_counter: a short interval, independent of the controller's clock
        start = time.perf_counter()
        result = STS_TIMEOUT
        while result == STS_TIMEOUT and time.perf_counter() - start < self.reply_timeout_s:
            for byte in self._receive():
                if not reply.feed(byte) or reply.equals(packet):
                    continue  # incomplete, or our own packet echoed by a one-wire adapter
                if reply.id == servo_id:
                    self.last_status = reply.error
                    result = STS_OK
                    break
        self.bad_packets += reply.bad_packets
        if result == STS_TIMEOUT and reply.bad_packets:
            result = STS_BAD_REPLY
        return result, reply

    def read_position(self, servo_id: int) -> tuple[str, int]:
        result, reply = self._transact(
            sts_read(servo_id, STS_REG_PRESENT_POSITION, 2), servo_id
        )
        if result != STS_OK:
            return result, 0
        if len(reply.params) != 2:
            return STS_BAD_REPLY, 0
        return STS_OK, sts_decode_position(reply.params[0] | reply.params[1] << 8)

    def write_torque(self, servo_id: int, enable: bool) -> str:
        packet = sts_write(servo_id, STS_REG_TORQUE_ENABLE, bytes((1 if enable else 0,)))
        return self._transact(packet, servo_id)[0]

    def write_torque_limit(self, servo_id: int, limit: int) -> str:
        """Torque limit, 0-1000 (0.1 %); RAM, until the next power-up."""
        packet = sts_write(servo_id, STS_REG_TORQUE_LIMIT, bytes((limit & 0xFF, limit >> 8)))
        return self._transact(packet, servo_id)[0]

    def sync_write_position(
        self, ids: Sequence[int], steps: Sequence[int], speed: int, acceleration: int
    ) -> None:
        # Broadcast: the servos do not reply
        self._send(sts_sync_write_position(ids, steps, speed, acceleration))


class StsJoint(NamedTuple):
    name: str  # ROS joint name: pan_joint, tilt_joint, actuator_joint, trigger_joint
    servo_id: int  # STS bus ID
    zero_steps: int  # servo position (0-4095) at joint position 0
    direction: int  # +1: steps increase with the joint position, -1: decrease
    lower: float  # joint limits enforced on every command (rad, or m for a slide);
    upper: float  # the lower one is the launcher joints' rest (safe) position
    # 0: revolute joint, positions in radians; > 0: a rack-and-pinion slide,
    # positions in metres of rack travel (the pinion's pitch radius)
    pinion_radius_m: float = 0.0
    torque_limit: int = 0  # 0: the servo's own; 1-1000 (0.1 %): written at every torque enable

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower), self.upper)

    @property
    def rest(self) -> float:
        """A launcher joint's rest (safe) position: its lower limit."""
        return self.lower

    def shaft_angle(self, value: float) -> float:
        return value / self.pinion_radius_m if self.pinion_radius_m > 0 else value

    def to_steps(self, value: float) -> int:
        """Joint position to servo steps, clamped to 0..4095 (not to the joint limits)."""
        steps = math.floor(
            self.zero_steps + self.direction * self.shaft_angle(value) * STS_STEPS_PER_RAD + 0.5
        )
        return min(max(steps, 0), STS_MAX_STEP)

    def from_steps(self, steps: int) -> float:
        angle = self.direction * (steps - self.zero_steps) / STS_STEPS_PER_RAD
        return angle * self.pinion_radius_m if self.pinion_radius_m > 0 else angle


class StsConfig(NamedTuple):
    """The firmware's Config (src/turret_config.h); joints are (pan, tilt),
    or (pan, tilt, rack, trigger) when the launcher is fitted. Without pan
    and tilt (pan_tilt_fitted false: a launcher alone, as on the aerodrone)
    the first two are placeholders the controller never touches."""

    joints: tuple[StsJoint, ...]
    speed: int = 2000  # goal speed, steps/s (0 = servo maximum)
    acceleration: int = 50  # goal acceleration, 100 steps/s^2 units
    watchdog_s: float = 0.5  # hold when no SET arrives for this long
    feedback_period_s: float = 0.04  # measured position report period (25 Hz)
    error_repeat_s: float = 1.0  # minimum interval between repeated servo errors
    torque_retry_s: float = 0.25  # minimum interval between torque re-enables
    stop_disables_torque: bool = False  # STOP/watchdog: torque off instead of holding
    pan_tilt_fitted: bool = True  # false: the launcher alone on the bus

    @property
    def actuator_fitted(self) -> bool:
        return len(self.joints) == 4

    @property
    def servo_indices(self) -> tuple[int, ...]:
        """The joints whose servos are on the bus."""
        return tuple(range(len(self.joints))) if self.pan_tilt_fitted else (2, 3)

    def validate(self) -> None:
        """Raise ValueError if the configuration is unusable."""
        if len(self.joints) not in (2, 4):
            raise ValueError("joints must be (pan, tilt) or (pan, tilt, rack, trigger)")
        for joint in self.joints:
            if joint.direction not in (1, -1):
                raise ValueError(f"{joint.name}: direction must be +1 or -1")
            if not 0 <= joint.servo_id <= 253:
                raise ValueError(f"{joint.name}: servo id must be 0-253")
            if not 0 <= joint.zero_steps <= STS_MAX_STEP:
                raise ValueError(f"{joint.name}: zero_steps must be 0-4095")
            if not (
                math.isfinite(joint.lower)
                and math.isfinite(joint.upper)
                and joint.lower < joint.upper
            ):
                raise ValueError(f"{joint.name}: joint limits must be finite with lower < upper")
            if not (math.isfinite(joint.pinion_radius_m) and joint.pinion_radius_m >= 0):
                raise ValueError(f"{joint.name}: pinion_radius_m must be finite and >= 0")
            if not 0 <= joint.torque_limit <= 1000:
                raise ValueError(f"{joint.name}: torque_limit must be 0-1000")
            # The limits must map inside one servo turn, or clamping to 0..4095
            # would silently move the joint somewhere else.
            for value in (joint.lower, joint.upper):
                steps = (joint.zero_steps
                         + joint.direction * joint.shaft_angle(value) * STS_STEPS_PER_RAD)
                if not -0.5 <= steps <= STS_MAX_STEP + 0.5:
                    raise ValueError(
                        f"{joint.name}: joint limits reach outside servo steps 0-4095"
                    )
        if self.joints[0].servo_id == self.joints[1].servo_id:
            raise ValueError("pan and tilt servo ids must differ")
        if self.actuator_fitted:
            if len({joint.servo_id for joint in self.joints}) != 4:
                raise ValueError(
                    "the launcher's servo ids must differ from each other and pan and tilt"
                )
            if not self.joints[2].pinion_radius_m > 0:
                raise ValueError("the rack (a slide) needs pinion_radius_m > 0")
        if not 0 <= self.speed <= 0x7FFF or not 0 <= self.acceleration <= 0xFF:
            raise ValueError("servo speed must be 0-32767 and acceleration 0-255")
        if self.watchdog_s <= 0 or self.feedback_period_s <= 0:
            raise ValueError("watchdog_s and feedback_period_s must be > 0")
        if not self.pan_tilt_fitted and not self.actuator_fitted:
            raise ValueError("without pan and tilt the bus needs the launcher's rack and trigger")


class StsController:
    """The firmware's Controller (src/turret_core.cpp) on this host.

    set() clamps to the joint limits and moves the servos in one sync write,
    enabling torque first on the first SET (and again, at most every
    torque_retry_s, after a servo stopped answering), with each joint's
    torque limit; a fitted launcher's trigger goes to the given angle or 0,
    its rack to the given position or stops where it is. stop(), and
    update() when no SET arrived for watchdog_s, hold pan and tilt's measured
    position, return the trigger to 0 and hold the rack where it is.
    update() reads the positions every feedback_period_s and reports them
    only when pan and tilt answered, with the launcher's when both its
    servos answered too. Servo errors are
    reported at once, then at most every error_repeat_s while they persist.
    Nothing is written to the servos before the first set() or stop(). Times
    are seconds on one clock.
    """

    IDLE = "idle"  # no SET yet: servos untouched
    ACTIVE = "active"  # following SET commands
    HOLDING = "holding"  # stopped by STOP or the watchdog until the next SET

    def __init__(
        self,
        config: StsConfig,
        bus: StsBus,
        on_position: Callable[..., None],
        on_error: Callable[[str], None],
    ) -> None:
        config.validate()
        self.config = config
        self.bus = bus
        # on_position(pan, tilt), or (pan, tilt, rack, trigger) when the launcher answered
        self.on_position = on_position
        self.on_error = on_error
        self.state = self.IDLE
        self.last_set_time = float("-inf")
        self.next_feedback_time = float("-inf")
        self.last_torque_attempt = float("-inf")
        self.torque_unknown = False  # a servo stopped answering: re-enable torque on the next SET
        self.have_feedback = False
        self.have_launcher = False  # both launcher servos answered at the last read
        count = len(config.joints)
        self.measured = [0.0] * count
        # Last commanded goal, after clamping (the launcher starts at rest)
        self.goal = [joint.rest if index >= 2 else 0.0
                     for index, joint in enumerate(config.joints)]
        self.joint_failing = [False] * count
        self.last_error_time = [float("-inf")] * count
        self.last_ignored_time = float("-inf")  # last "no launcher fitted" report

    @property
    def ids(self) -> list[int]:
        return [joint.servo_id for joint in self.config.joints]

    def set(
        self,
        pan: float,
        tilt: float,
        now: float,
        actuator: float | None = None,
        trigger: float | None = None,
    ) -> bool:
        """Clamp to the joint limits and command the servos; False for
        non-finite values. A fitted launcher's trigger goes to ``trigger``
        (0 when None) and its rack to ``actuator`` (stopped where it is, its
        present position, when None)."""
        if not (
            math.isfinite(pan)
            and math.isfinite(tilt)
            and (actuator is None or math.isfinite(actuator))
            and (trigger is None or math.isfinite(trigger))
        ):
            self.on_error("rejected non-finite SET")
            return False
        joints = self.config.joints
        if (
            (actuator is not None or trigger is not None)
            and not self.config.actuator_fitted
            and now - self.last_ignored_time >= self.config.error_repeat_s
        ):
            self.on_error(
                "actuator_joint/trigger_joint ignored: no launcher fitted (turret_config.h)"
            )
            self.last_ignored_time = now
        self.goal[0], self.goal[1] = joints[0].clamp(pan), joints[1].clamp(tilt)
        if self.state != self.ACTIVE or (
            self.torque_unknown
            and now - self.last_torque_attempt >= self.config.torque_retry_s
        ):
            self._enable_torque(now)
        ids, steps = [], []
        if self.config.pan_tilt_fitted:
            ids = [joints[0].servo_id, joints[1].servo_id]
            steps = [joints[0].to_steps(self.goal[0]), joints[1].to_steps(self.goal[1])]
        if self.config.actuator_fitted:
            # The trigger leaves 0 only while SETs keep naming it; the rack
            # moves only while SETs keep naming it and is otherwise stopped
            # where it is.
            rack, trigger_joint = joints[2], joints[3]
            self.goal[3] = trigger_joint.rest if trigger is None else trigger_joint.clamp(trigger)
            if actuator is not None:
                self.goal[2] = rack.clamp(actuator)
                ids.append(rack.servo_id)
                steps.append(rack.to_steps(self.goal[2]))
            else:
                present = self._read_joint(2, now)
                if present is not None:
                    self.goal[2] = self.measured[2]
                    ids.append(rack.servo_id)
                    steps.append(present)
            ids.append(trigger_joint.servo_id)
            steps.append(trigger_joint.to_steps(self.goal[3]))
        self.bus.sync_write_position(ids, steps, self.config.speed, self.config.acceleration)
        self.state = self.ACTIVE
        self.last_set_time = now
        return True

    def stop(self, now: float) -> None:
        """Stop moving: hold pan and tilt's measured position (or disable
        their torque), return the trigger to 0 and hold the rack where it is."""
        self._hold(now)
        self.state = self.HOLDING

    def update(self, now: float) -> None:
        """Run the watchdog and the periodic position feedback."""
        if self.state == self.ACTIVE and now - self.last_set_time >= self.config.watchdog_s:
            # No SET for watchdog_s (tracker or bridge input gone): stop where
            # we are, the trigger back to 0.
            self._hold(now)
            self.state = self.HOLDING
        if now >= self.next_feedback_time:
            # Keep the phase, so a poll timer at twice the rate gives the full 25 Hz
            self.next_feedback_time += self.config.feedback_period_s
            if self.next_feedback_time <= now:
                self.next_feedback_time = now + self.config.feedback_period_s
            self._read_feedback(now)

    def _read_joint(self, index: int, now: float) -> int | None:
        joint = self.config.joints[index]
        result, steps = self.bus.read_position(joint.servo_id)
        if result != STS_OK:
            self.torque_unknown = True  # it may have lost power; re-enable on the next SET
            self._report(index, result, now)
            self.joint_failing[index] = True
            return None
        status = self.bus.last_status
        if status:
            names = "".join(f" {name}" for bit, name in STS_FAULTS if status & bit)
            self._report(index, f"fault 0x{status:02X}{names}", now)
            self.joint_failing[index] = True
        else:
            self.joint_failing[index] = False  # recovered: the next failure reports at once
        self.measured[index] = joint.from_steps(steps)
        return steps

    def _read_feedback(self, now: float) -> None:
        if not self.config.pan_tilt_fitted:
            # The launcher alone: reported when both its servos answered.
            launcher = [self._read_joint(index, now) is not None for index in (2, 3)]
            self.have_launcher = self.have_feedback = all(launcher)
            if self.have_feedback:
                self.on_position(None, None, *self.measured[2:4])
            return
        answered = [self._read_joint(index, now) is not None for index in (0, 1)]
        self.have_launcher = False
        if self.config.actuator_fitted:
            launcher = [self._read_joint(index, now) is not None for index in (2, 3)]
            self.have_launcher = all(launcher)
        self.have_feedback = all(answered)
        if self.have_feedback:
            # Never report a joint that did not answer: the host treats POS as
            # measured. A silent launcher servo leaves pan and tilt reported
            # without the launcher.
            self.on_position(*self.measured[:4 if self.have_launcher else 2])

    def _enable_torque(self, now: float) -> None:
        self.last_torque_attempt = now
        ok = True
        for index, joint in enumerate(self.config.joints):
            if index < 2 and not self.config.pan_tilt_fitted:
                continue
            result = self.bus.write_torque(joint.servo_id, True)
            what = "torque enable"
            if result == STS_OK and joint.torque_limit > 0:
                # RAM register: the servo forgets it when it loses power
                result = self.bus.write_torque_limit(joint.servo_id, joint.torque_limit)
                what = "torque limit"
            if result != STS_OK:
                self._report(index, f"{what} {result}", now)
                ok = False
        self.torque_unknown = not ok

    def _hold(self, now: float) -> None:
        ids: list[int] = []
        steps: list[int] = []
        for index, joint in enumerate(self.config.joints[:2 if self.config.pan_tilt_fitted else 0]):
            if self.config.stop_disables_torque:
                result = self.bus.write_torque(joint.servo_id, False)
                if result != STS_OK:
                    self._report(index, f"torque disable {result}", now, force=True)
                continue
            # Hold the measured position: a goal equal to the present position.
            present = self._read_joint(index, now)
            if present is None:
                self._report(index, "cannot hold: no position reply", now, force=True)
            else:
                ids.append(joint.servo_id)
                steps.append(present)
        if self.config.actuator_fitted:
            # The launcher's safe state, whatever pan and tilt do: the rack
            # stopped where it is, the trigger at 0.
            rack, trigger = self.config.joints[2], self.config.joints[3]
            present = self._read_joint(2, now)
            if present is None:
                self._report(2, "cannot hold: no position reply", now, force=True)
            else:
                self.goal[2] = self.measured[2]
                ids.append(rack.servo_id)
                steps.append(present)
            self.goal[3] = trigger.rest
            ids.append(trigger.servo_id)
            steps.append(trigger.to_steps(trigger.rest))
        if ids:
            self.bus.sync_write_position(
                ids, steps, self.config.speed, self.config.acceleration
            )

    def _report(self, index: int, what: str, now: float, force: bool = False) -> None:
        if (
            not force
            and self.joint_failing[index]
            and now - self.last_error_time[index] < self.config.error_repeat_s
        ):
            return
        joint = self.config.joints[index]
        self.on_error(f"servo {joint.servo_id} ({joint.name}) {what}")
        self.last_error_time[index] = now


class TurretArduinoBridge(Node):
    # Transport and actuator state defaults; __init__ sets them from the parameters.
    transport = "serial"
    router = None
    mcu_ready = False
    registered: frozenset[str] | set[str] = frozenset()
    servos: StsController | None = None
    actuator_fitted = False  # the dart launcher: rack (actuator_joint) and trigger
    pan_tilt_fitted = True  # false: the launcher alone (sts), as on the aerodrone
    actuator_min_m = 0.0
    actuator_max_m = 0.038
    trigger_min_rad = 0.0
    trigger_max_rad = 0.6
    launcher_position: tuple[float, float] | None = None  # last measured (rack, trigger)
    ignored_actuator_commands = 0

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
        # MCU through the arduino-router socket (run on the UNO Q itself);
        # sts: the servos themselves on serial_port (1000000 baud), no Arduino
        self.declare_parameter(
            "transport", os.environ.get("TURRET_ARDUINO_TRANSPORT", "serial")
        )
        self.declare_parameter(
            "router_socket", os.environ.get("TURRET_ROUTER_SOCKET", ROUTER_SOCKET)
        )
        # uno_q_bridge: an RPC unanswered for this long marks the MCU not ready
        self.declare_parameter("rpc_timeout_s", 0.5)
        # uno_q_bridge: exit (status 1) when the router socket has been
        # unreachable this long, 0 to keep retrying. A bind-mounted socket file
        # goes stale when arduino-router restarts and recreates it; the
        # container's restart policy then starts the bridge again on the new one.
        self.declare_parameter("router_lost_exit_s", 30.0)
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
        # sts: the firmware's src/turret_config.h settings (pan_tilt_ros
        # convention: steps = 2048 - rad * 4096 / 2pi, IDs of its ctrl_config.yaml)
        self.declare_parameter("pan_servo_id", 12)
        self.declare_parameter("tilt_servo_id", 13)
        self.declare_parameter("pan_zero_steps", 2048)
        self.declare_parameter("tilt_zero_steps", 2048)
        self.declare_parameter("pan_direction", -1)
        self.declare_parameter("tilt_direction", -1)
        self.declare_parameter("servo_speed", 2000)  # steps/s
        self.declare_parameter("servo_acceleration", 50)  # 100 steps/s^2
        self.declare_parameter("servo_reply_timeout_s", 0.01)
        self.declare_parameter("stop_disables_torque", False)
        # The foam dart launcher (its priming rack, a third ST3215, and its
        # trigger servo): TURRET_ACTUATOR_FITTED in the env files, false by
        # default. The rack's limits are metres of travel (0 forward), the
        # trigger's radians (0 safe); the sts transport also uses the servo
        # settings (the firmware's turret_config.h).
        self.declare_parameter(
            "actuator_fitted", env_bool("TURRET_ACTUATOR_FITTED", False)
        )
        # The launcher alone, no pan and tilt (sts, actuator_fitted): the
        # aerodrone's downward launcher. TURRET_PAN_TILT_FITTED, true by default.
        self.declare_parameter(
            "pan_tilt_fitted", env_bool("TURRET_PAN_TILT_FITTED", True)
        )
        self.declare_parameter("actuator_min_m", 0.0)
        self.declare_parameter("actuator_max_m", 0.038)
        self.declare_parameter("actuator_servo_id", 14)
        self.declare_parameter("actuator_zero_steps", 512)
        self.declare_parameter("actuator_direction", 1)
        self.declare_parameter("actuator_pinion_radius_m", 0.0135)
        self.declare_parameter("actuator_torque_limit", 550)  # 0.1 %: 55 %
        self.declare_parameter("trigger_min_rad", 0.0)
        self.declare_parameter("trigger_max_rad", 0.6)
        self.declare_parameter("trigger_servo_id", 15)
        self.declare_parameter("trigger_zero_steps", 2048)
        self.declare_parameter("trigger_direction", 1)
        self.declare_parameter("trigger_torque_limit", 300)  # 0.1 %: 30 %

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
        self.router_lost_exit_s = float(self.get_parameter("router_lost_exit_s").value)
        self.router_down_since: float | None = None
        self.router: RouterBridgeLink | None = None
        self.mcu_ready = False
        self.registered = set()  # notifications the router now routes to us
        self.last_probe_time = float("-inf")
        self.probe_pending = False
        self.pan_min_rad = float(self.get_parameter("pan_min_rad").value)
        self.pan_max_rad = float(self.get_parameter("pan_max_rad").value)
        self.tilt_min_rad = float(self.get_parameter("tilt_min_rad").value)
        self.tilt_max_rad = float(self.get_parameter("tilt_max_rad").value)
        self.actuator_fitted = bool(self.get_parameter("actuator_fitted").value)
        self.pan_tilt_fitted = bool(self.get_parameter("pan_tilt_fitted").value)
        if not self.pan_tilt_fitted and not (self.transport == "sts" and self.actuator_fitted):
            raise ValueError(
                "pan_tilt_fitted false (the launcher alone) needs the sts transport "
                "and actuator_fitted true"
            )
        self.actuator_min_m = float(self.get_parameter("actuator_min_m").value)
        self.actuator_max_m = float(self.get_parameter("actuator_max_m").value)
        self.trigger_min_rad = float(self.get_parameter("trigger_min_rad").value)
        self.trigger_max_rad = float(self.get_parameter("trigger_max_rad").value)
        if self.actuator_fitted and not all(
            math.isfinite(low) and math.isfinite(high) and low < high
            for low, high in ((self.actuator_min_m, self.actuator_max_m),
                              (self.trigger_min_rad, self.trigger_max_rad))
        ):
            raise ValueError("the launcher's limits must be finite with min < max")
        self.launcher_position = None
        self.ignored_actuator_commands = 0
        self.servos: StsController | None = None
        self.servo_reply_timeout_s = float(self.get_parameter("servo_reply_timeout_s").value)
        self.sts_config = self._sts_config() if self.transport == "sts" else None
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
        elif self.transport == "sts":
            names = ", ".join(
                f"{self.sts_config.joints[index].servo_id} "
                f"({self.sts_config.joints[index].name.split('_')[0]})"
                for index in self.sts_config.servo_indices
            )
            self.get_logger().info(
                f"Turret bridge driving STS servos {names} on {self.serial_port} "
                f"at {self.baud_rate} baud"
            )
        else:
            self.get_logger().info(
                "Turret Arduino bridge ready on "
                f"{self.serial_port} at {self.baud_rate} baud"
            )

    def _sts_config(self) -> StsConfig:
        """Servo settings from the parameters; ValueError if unusable."""

        def param(name: str) -> object:
            return self.get_parameter(name).value

        joints = tuple(
            StsJoint(
                name=f"{side}_joint",
                servo_id=int(param(f"{side}_servo_id")),
                zero_steps=int(param(f"{side}_zero_steps")),
                direction=int(param(f"{side}_direction")),
                lower=getattr(self, f"{side}_min_rad"),
                upper=getattr(self, f"{side}_max_rad"),
            )
            for side in ("pan", "tilt")
        )
        if self.actuator_fitted:
            joints += (
                StsJoint(
                    name=ACTUATOR_JOINT,
                    servo_id=int(param("actuator_servo_id")),
                    zero_steps=int(param("actuator_zero_steps")),
                    direction=int(param("actuator_direction")),
                    lower=self.actuator_min_m,
                    upper=self.actuator_max_m,
                    pinion_radius_m=float(param("actuator_pinion_radius_m")),
                    torque_limit=int(param("actuator_torque_limit")),
                ),
                StsJoint(
                    name=TRIGGER_JOINT,
                    servo_id=int(param("trigger_servo_id")),
                    zero_steps=int(param("trigger_zero_steps")),
                    direction=int(param("trigger_direction")),
                    lower=self.trigger_min_rad,
                    upper=self.trigger_max_rad,
                    torque_limit=int(param("trigger_torque_limit")),
                ),
            )
        config = StsConfig(
            joints=joints,
            speed=int(param("servo_speed")),
            acceleration=int(param("servo_acceleration")),
            stop_disables_torque=bool(param("stop_disables_torque")),
            pan_tilt_fitted=self.pan_tilt_fitted,
        )
        config.validate()
        return config

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
        sts = self.transport == "sts"
        options = {"timeout": 0.05, "write_timeout": 0.1}
        if sts:
            # Short reads: the bus waits for each servo's reply itself.
            # Exclusive (an advisory lock): a second bridge on the same bus
            # would interleave its packets with ours.
            options.update(timeout=STS_READ_TIMEOUT_S, exclusive=True)
        try:
            self.serial_handle = serial.Serial(self.serial_port, self.baud_rate, **options)
            # Non-blocking settle: skip writes until the Arduino has rebooted.
            # The servos (sts) do not reset when the port opens.
            self.serial_ready_time = now + (0.0 if sts else self.connect_settle_s)
            self.rx_buffer = b""
            self.stop_sent = False
            self.last_error = ""
            if sts:
                # A fresh controller, like the firmware at power-up: idle, and
                # nothing written to the servos before the first SET or STOP
                self.servos = StsController(
                    self.sts_config,
                    StsBus(self.serial_handle, self.servo_reply_timeout_s),
                    self._publish_joint_state,
                    self._servo_error,
                )
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
            self._router_down(now)
            return
        self.router_down_since = None
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

    def _router_down(self, now: float) -> None:
        """Count how long the router has been unreachable; give up past the limit."""
        if self.router_down_since is None:
            self.router_down_since = now
        down_s = now - self.router_down_since
        if 0.0 < self.router_lost_exit_s <= down_s:
            raise RouterUnreachable(
                f"arduino-router socket {self.router_socket} unreachable for "
                f"{down_s:.0f} s ({self.last_error}); exiting so the container "
                "restarts on the current socket"
            )

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
                if method == RPC_SET_ALL:
                    self.last_error += " (a build without the launcher has no turret/set_all)"
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
        if method in (RPC_SET, RPC_SET_ALL) and result is False:
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
        if self.transport == "sts" and self.servos is None:
            return False
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

    def _send_set(
        self, pan: float, tilt: float, launcher: tuple[float, float] | None = None
    ) -> bool:
        """SET pan and tilt, and the launcher's (rack, trigger) unless it is
        None: then the firmware returns the trigger to 0 and stops the rack."""
        if self.transport == "uno_q_bridge":
            if launcher is None:
                return self._send_rpc(RPC_SET, (pan, tilt))
            return self._send_rpc(RPC_SET_ALL, (pan, tilt, *launcher))
        if self.transport == "sts":
            actuator, trigger = launcher if launcher is not None else (None, None)
            return self._send_servos(
                lambda now: self.servos.set(pan, tilt, now, actuator, trigger)
            )
        return self._write(format_command(pan, tilt, launcher))

    def _send_stop(self) -> bool:
        if self.transport == "uno_q_bridge":
            return self._send_rpc(RPC_STOP, ())
        if self.transport == "sts":
            return self._send_servos(lambda now: self.servos.stop(now))
        return self._write(STOP_LINE)

    def _send_servos(self, action: Callable[[float], object]) -> bool:
        # Like a successful serial write, a command clears last_error; servo
        # errors it causes (torque enable, hold) are reported after this
        self.last_error = ""
        return self._servo_io(action)

    def _servo_io(self, action: Callable[[float], object]) -> bool:
        """Run a controller step on the servo bus; False if the port failed."""
        try:
            action(time.monotonic())
        except StsLinkError as exc:
            self.last_error = str(exc)
            self._close_serial()
            return False
        return True

    def _servo_error(self, text: str) -> None:
        # The firmware's ERR reports, from the controller running here
        self.last_error = text

    def _send_rpc(self, method: str, params: tuple) -> bool:
        # Like a successful serial write, a delivered command clears last_error
        sent = self._rpc(method, params)
        if sent:
            self.last_error = ""
        return sent

    def _joint_command(self, msg: JointState) -> None:
        pan = self._joint_position(msg, "pan_joint")
        tilt = self._joint_position(msg, "tilt_joint")
        actuator = self._joint_position(msg, ACTUATOR_JOINT)
        trigger = self._joint_position(msg, TRIGGER_JOINT)
        if not self.pan_tilt_fitted:
            pan = tilt = 0.0  # no pan and tilt servos: ignored
        if pan is None or tilt is None:
            self.last_error = "joint command must include pan_joint and tilt_joint"
            return
        values = [value for value in (pan, tilt, actuator, trigger) if value is not None]
        if not all(math.isfinite(value) for value in values):
            self.last_error = f"rejected non-finite joint command {tuple(values)}"
            self.get_logger().warning(self.last_error, throttle_duration_sec=5.0)
            return
        if not self.actuator_ready():
            # Never queue: a command held through an outage or e-stop would be
            # executed as a full-speed jump when the link comes back.
            self.dropped_commands += 1
            return
        if self.pan_tilt_fitted:
            pan = min(max(pan, self.pan_min_rad), self.pan_max_rad)
            tilt = min(max(tilt, self.tilt_min_rad), self.tilt_max_rad)
        launcher = None
        if self.actuator_fitted:
            if actuator is not None:
                # The command's rack position and trigger angle (0 when it does
                # not carry one); without actuator_joint the SET carries pan and
                # tilt only: trigger to 0, rack stopped where it is.
                launcher = (
                    min(max(actuator, self.actuator_min_m), self.actuator_max_m),
                    min(max(self.trigger_min_rad if trigger is None else trigger,
                            self.trigger_min_rad), self.trigger_max_rad),
                )
        elif actuator is not None or trigger is not None:
            self.ignored_actuator_commands += 1
            self.get_logger().warning(
                "ignoring the launcher's joints: actuator_fitted is false",
                throttle_duration_sec=10.0,
            )
        if self._send_set(pan, tilt, launcher):
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
        if self.transport == "sts":
            # Watchdog and position feedback, as the firmware's loop()
            if not self._servo_io(self.servos.update):
                return
        else:
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

    def _publish_joint_state(
        self,
        pan: float | None,
        tilt: float | None,
        actuator: float | None = None,
        trigger: float | None = None,
    ) -> None:
        """Measured positions; the launcher's only when it is fitted and both
        its servos answered; pan and tilt None without them."""
        self.last_feedback_time = time.monotonic()
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(JOINT_NAMES) if pan is not None else []
        state.position = [pan, tilt] if pan is not None else []
        if self.actuator_fitted:
            launcher = None if actuator is None or trigger is None else (actuator, trigger)
            self.launcher_position = launcher
            if launcher is not None:
                # (extend: rclpy keeps position in an array.array)
                state.name.extend([ACTUATOR_JOINT, TRIGGER_JOINT])
                state.position.extend(launcher)
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
        sts = self.transport == "sts"
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
            if uno_q:
                link = "UNO Q MCU connected"
            elif sts:
                link = "servo bus connected"
            else:
                link = "serial connected"
            source = "servos" if sts else "firmware"
            status.message = (
                link if feedback else f"{link}, no position feedback from {source}"
            )
        if uno_q:
            status.values = [KeyValue(key="router_socket", value=self.router_socket)]
        else:
            status.values = [
                KeyValue(key="port", value=self.serial_port),
                KeyValue(key="baud_rate", value=str(self.baud_rate)),
            ]
        if sts:
            ids = ",".join(str(joint.servo_id) for joint in self.sts_config.joints)
            servos = self.servos
            status.values += [
                KeyValue(key="servo_ids", value=ids),
                KeyValue(key="servo_state", value=servos.state if servos else "disconnected"),
                KeyValue(key="bad_packets", value=str(servos.bus.bad_packets if servos else 0)),
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
            KeyValue(key="actuator_fitted", value=str(self.actuator_fitted).lower()),
        ]
        if not self.pan_tilt_fitted:
            status.values.append(KeyValue(key="pan_tilt_fitted", value="false"))
        if self.actuator_fitted:
            position = self.launcher_position if feedback else None
            status.values += [
                KeyValue(key="actuator_position_m",
                         value="unknown" if position is None else f"{position[0]:.4f}"),
                KeyValue(key="trigger_position_rad",
                         value="unknown" if position is None else f"{position[1]:.3f}"),
            ]
        elif self.ignored_actuator_commands:
            status.values.append(KeyValue(
                key="ignored_actuator_commands", value=str(self.ignored_actuator_commands)
            ))
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)

    def _close_serial(self) -> None:
        self.servos = None
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
    status = 0
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except RouterUnreachable as exc:
        node.get_logger().error(str(exc))
        status = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if status:
        sys.exit(status)


if __name__ == "__main__":
    main()

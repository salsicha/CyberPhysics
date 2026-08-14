#!/usr/bin/env python3
"""ArduPilot SITL JSON transport and ENU/FLU conversion helpers.

The simulator is authoritative for dynamics.  ArduPilot sends motor PWM over
UDP and receives the resulting physical state in its documented JSON backend
format.  This module deliberately has no ROS or simulator dependency so all
three backends use the identical controller boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import struct
import time
from typing import Iterable

import numpy as np


MAGIC_16 = 18458
MAGIC_32 = 29569
PACKET_16 = struct.Struct("<HHI16H")
PACKET_32 = struct.Struct("<HHI32H")


@dataclass(frozen=True)
class ServoFrame:
    frame_rate_hz: int
    frame_count: int
    pwm: tuple[int, ...]


class ArduPilotJSON:
    """Small server for ArduPilot's lock-step external physics protocol."""

    def __init__(self, bind_host: str = "0.0.0.0", port: int = 9002):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind_host, port))
        self.socket.setblocking(False)
        self.peer = None
        self.last_frame = None

    def receive(self) -> ServoFrame | None:
        """Drain queued packets and return the newest valid actuator frame."""
        newest = None
        while True:
            try:
                payload, peer = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            frame = decode_servo_packet(payload)
            if frame is not None:
                newest = frame
                self.peer = peer
        if newest is not None:
            self.last_frame = newest
        return newest

    def send(self, state: dict) -> bool:
        if self.peer is None:
            return False
        payload = json.dumps(state, separators=(",", ":"), allow_nan=False)
        self.socket.sendto((payload + "\n").encode("ascii"), self.peer)
        return True

    def close(self):
        self.socket.close()


def decode_servo_packet(payload: bytes) -> ServoFrame | None:
    if len(payload) == PACKET_16.size:
        values = PACKET_16.unpack(payload)
        expected_magic = MAGIC_16
    elif len(payload) == PACKET_32.size:
        values = PACKET_32.unpack(payload)
        expected_magic = MAGIC_32
    else:
        return None
    if values[0] != expected_magic:
        return None
    return ServoFrame(values[1], values[2], tuple(values[3:]))


def pwm_to_rotor_speed(
    pwm: Iterable[int], maximum_speed: float, order=(0, 1, 2, 3)
) -> np.ndarray:
    """Convert PWM thrust commands to rotor angular speed/RPM.

    The motor model consumes speed while ArduPilot's output is proportional to
    thrust.  Taking sqrt preserves that relationship instead of treating PWM
    as angular speed, which would give the wrong thrust curve.
    """
    values = np.asarray(tuple(pwm), dtype=np.float64)
    thrust = np.clip((values - 1000.0) / 1000.0, 0.0, 1.0)
    speeds = maximum_speed * np.sqrt(thrust)
    return speeds[np.asarray(order, dtype=np.int64)]


def quaternion_to_matrix(quaternion_wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return np.eye(3)
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix) -> list[float]:
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                 (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                 0.25 * s, (m[1, 2] + m[2, 1]) / s]
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                 (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.asarray(q, dtype=np.float64)
    q /= np.linalg.norm(q)
    return q.tolist()


def enu_flu_to_ned_frd(quaternion_wxyz) -> list[float]:
    world_enu_to_ned = np.array([[0.0, 1.0, 0.0],
                                 [1.0, 0.0, 0.0],
                                 [0.0, 0.0, -1.0]])
    body_frd_to_flu = np.diag([1.0, -1.0, -1.0])
    rotation = world_enu_to_ned @ quaternion_to_matrix(quaternion_wxyz) @ body_frd_to_flu
    return matrix_to_quaternion(rotation)


def latlon_from_enu(origin_lat, origin_lon, east_m, north_m):
    latitude = origin_lat + north_m / 111_320.0
    longitude = origin_lon + east_m / (
        111_320.0 * math.cos(math.radians(origin_lat)))
    return latitude, longitude


def state_from_enu(
    position_enu,
    velocity_enu,
    quaternion_wxyz,
    gyro_flu,
    accel_flu,
    origin_lat,
    origin_lon,
    origin_alt,
    timestamp=None,
    reference_enu=(0.0, 0.0, 0.0),
) -> dict:
    east, north, up = (float(v) for v in position_enu)
    reference_east, reference_north, reference_up = (
        float(v) for v in reference_enu)
    ve, vn, vu = (float(v) for v in velocity_enu)
    latitude, longitude = latlon_from_enu(
        origin_lat, origin_lon, east, north)
    gyro_frd = np.diag([1.0, -1.0, -1.0]) @ np.asarray(gyro_flu, dtype=np.float64)
    accel_frd = np.diag([1.0, -1.0, -1.0]) @ np.asarray(accel_flu, dtype=np.float64)
    return {
        "timestamp": float(time.time() if timestamp is None else timestamp),
        "imu": {
            "gyro": gyro_frd.tolist(),
            "accel_body": accel_frd.tolist(),
        },
        # ArduPilot JSON position is NED relative to SITL home; simulator poses
        # are in the terrain-origin ENU frame.
        "position": [north - reference_north, east - reference_east,
                     -(up - reference_up)],
        "velocity": [vn, ve, -vu],
        "quaternion": enu_flu_to_ned_frd(quaternion_wxyz),
        "lat": float(latitude),
        "lon": float(longitude),
        "alt": float(origin_alt + up),
    }


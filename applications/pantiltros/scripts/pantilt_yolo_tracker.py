#!/usr/bin/env python3
"""Shared ROS 2 YOLO perception and closed-loop control for PanTiltROS.

With ``actuator_fitted`` the turret also carries the foam dart launcher
(systems/racecar_turret/interfaces.md, "Foam dart launcher"): a priming rack
(``actuator_joint``, metres, 0 forward) and a trigger (``trigger_joint``,
radians, 0 safe), fired once per target. Only a ``{"action": "tag",
"target_id": ...}`` command naming the locked target, while every gate
holds, primes and fires it (FireCycle.start, called from _tag alone); every
other event leaves the trigger at 0 and the rack forward or stopped.
"""

from __future__ import annotations

import json
import math
import os
import time
from bisect import bisect_right
from collections import deque
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from ultralytics import YOLO

try:  # only base-rotation compensation needs it (the IMU's mounting)
    import tf2_ros
except ImportError:
    tf2_ros = None


IDENTITY = np.eye(3)
ACTUATOR_JOINT = "actuator_joint"  # the launcher's priming rack, metres, 0 forward
TRIGGER_JOINT = "trigger_joint"  # the launcher's trigger, radians, 0 safe
# How close the measured rack and trigger must come to a goal to count as there
RACK_TOLERANCE_M = 0.002
TRIGGER_TOLERANCE_RAD = 0.05
# While the launcher is not verified at rest, commands carrying its goals go
# out at least this often (the bridge's watchdog makes it safe after 0.5 s).
LAUNCHER_KEEPALIVE_S = 0.1


def clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        raise ValueError(f"cannot clamp non-finite value {value!r}")
    return max(low, min(high, value))


def finite(*values: Any) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def rotation_vector_matrix(vector: np.ndarray) -> np.ndarray:
    """Rotation by |vector| radians about vector (Rodrigues)."""
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return IDENTITY
    x, y, z = vector / angle
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return IDENTITY + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def joint_direction(pan: float, tilt: float) -> np.ndarray:
    """Optical axis in turret/base_link: pan_joint turns about -z (positive
    right), tilt_joint about the pan link's -y (positive up)."""
    return np.array(
        [
            math.cos(pan) * math.cos(tilt),
            -math.sin(pan) * math.cos(tilt),
            math.sin(tilt),
        ]
    )


def direction_joints(direction: np.ndarray) -> tuple[float, float]:
    """Inverse of joint_direction: the (pan, tilt) that point along direction."""
    x, y, z = (float(value) for value in direction)
    return math.atan2(-y, x), math.atan2(z, math.hypot(x, y))


def camera_ray(pan: float, tilt: float, err_pan: float, err_tilt: float) -> np.ndarray:
    """Line of sight in turret/base_link to a point seen at angular image
    errors (right and up positive) with the turret at (pan, tilt)."""
    # camera_link (x forward, y left, z up) turned by Rz(-pan) Ry(-tilt)
    ray = np.array([1.0, -math.tan(err_pan), math.tan(err_tilt)])
    cp, sp, ct, st = math.cos(pan), math.sin(pan), math.cos(tilt), math.sin(tilt)
    pan_rotation = np.array([[cp, sp, 0.0], [-sp, cp, 0.0], [0.0, 0.0, 1.0]])
    tilt_rotation = np.array([[ct, 0.0, -st], [0.0, 1.0, 0.0], [st, 0.0, ct]])
    return unit(pan_rotation @ tilt_rotation @ ray)


def fixed_attitude(_t: float) -> np.ndarray:
    return IDENTITY


Attitude = Callable[[float], np.ndarray]


class BaseRotation:
    """Orientation of turret/base_link in a world-fixed frame, from gyro rates.

    Integrating the base's angular velocity gives the rotation between any two
    recent instants, which is all the aim needs: the absolute heading of the
    integration frame, and its slow gyro drift, do not matter because the
    camera closes the loop. Past a sample the rotation continues at that
    sample's rate for at most ``extrapolation_s``, so when the rates stop
    arriving the attitude freezes and aiming falls back to a fixed base.
    """

    def __init__(
        self,
        timeout_s: float = 0.5,
        extrapolation_s: float = 0.25,
        history_s: float = 2.0,
        max_rate_rad_s: float = 10.0,
    ) -> None:
        self.timeout_s = float(timeout_s)
        self.extrapolation_s = float(extrapolation_s)
        self.history_s = float(history_s)
        self.max_rate = float(max_rate_rad_s)
        self.samples: deque[tuple[float, np.ndarray, np.ndarray]] = deque()

    def add(self, t: float, omega: Any) -> bool:
        """Add the base's angular velocity (rad/s, turret/base_link axes)."""
        omega = np.asarray(omega, dtype=float).reshape(3)
        if not finite(t, *omega) or np.linalg.norm(omega) > self.max_rate:
            return False
        if self.samples and t < self.samples[-1][0]:
            self.samples.clear()  # source restarted or the clock jumped back
        if self.samples and t == self.samples[-1][0]:
            return True
        rotation = IDENTITY
        if self.samples:
            last_t, last_rotation, last_omega = self.samples[-1]
            if t - last_t <= self.timeout_s:
                step = 0.5 * (last_omega + omega) * (t - last_t)
            else:  # across a gap, as attitude() extrapolates
                step = last_omega * self.extrapolation_s
            rotation = last_rotation @ rotation_vector_matrix(step)
        self.samples.append((float(t), rotation, omega))
        while len(self.samples) > 2 and t - self.samples[1][0] > self.history_s:
            self.samples.popleft()
        return True

    def fresh(self, now: float) -> bool:
        return bool(self.samples) and now - self.samples[-1][0] <= self.timeout_s

    def attitude(self, t: float) -> np.ndarray:
        """Rotation from turret/base_link at time t into the world-fixed frame."""
        if not self.samples:
            return IDENTITY
        times = [sample[0] for sample in self.samples]
        index = bisect_right(times, t)
        if index == 0:
            return self.samples[0][1]
        t0, rotation, omega = self.samples[index - 1]
        if index < len(self.samples):
            t1, _, next_omega = self.samples[index]
            if t1 - t0 <= self.timeout_s:
                return rotation @ rotation_vector_matrix(
                    0.5 * (omega + next_omega) * (t - t0)
                )
        return rotation @ rotation_vector_matrix(
            omega * min(t - t0, self.extrapolation_s)
        )


class BearingRate:
    """Rate of change of the target's line of sight (a unit vector in the
    world-fixed frame) from a least-squares line through the last
    ``window_s`` of detections.

    Fed forward, it removes the lag of the aim filter and of the
    capture-to-actuation latency for a target whose bearing changes steadily:
    a walking target, or the car driving past a still one.
    """

    def __init__(
        self, window_s: float = 0.5, min_samples: int = 4, max_rate_rad_s: float = 1.5
    ) -> None:
        self.window_s = float(window_s)
        self.min_samples = int(min_samples)
        self.max_rate = float(max_rate_rad_s)
        self.samples: deque[tuple[float, np.ndarray]] = deque()

    def reset(self) -> None:
        self.samples.clear()

    def add(self, t: float, sight: np.ndarray) -> None:
        if not finite(t, *sight):
            return
        if self.samples and t <= self.samples[-1][0]:
            if t == self.samples[-1][0]:
                return
            self.samples.clear()
        self.samples.append((float(t), np.asarray(sight, dtype=float)))
        while self.samples and t - self.samples[0][0] > self.window_s:
            self.samples.popleft()

    def rate(self, now: float) -> np.ndarray | None:
        """Line-of-sight rate (1/s), or None without enough recent samples."""
        if len(self.samples) < self.min_samples or not (
            0.0 <= now - self.samples[-1][0] <= self.window_s
        ):
            return None
        times = np.array([sample[0] for sample in self.samples])
        sights = np.array([sample[1] for sample in self.samples])
        offsets = times - times.mean()
        spread = float(offsets @ offsets)
        if spread < 1e-6:
            return None
        slope = offsets @ (sights - sights.mean(axis=0)) / spread
        last = sights[-1]
        slope = slope - float(slope @ last) * last  # keep it on the unit sphere
        speed = float(np.linalg.norm(slope))
        if speed > self.max_rate:
            slope *= self.max_rate / speed
        return slope


class DetectionTracks:
    """Stable string IDs for detections across frames.

    A detection continues the nearest track of its class whose last box
    centre lies within ``gate`` box diagonals; tracks survive ``timeout_s``
    without detections so an object keeps its ID through short occlusions.
    """

    def __init__(self, timeout_s: float = 1.0, gate: float = 1.0) -> None:
        self.timeout_s = float(timeout_s)
        self.gate = float(gate)
        self.tracks: dict[str, dict[str, Any]] = {}
        self.next_id = 1
        self.last_time = -math.inf

    def get(self, track_id: str) -> dict[str, Any] | None:
        return self.tracks.get(track_id)

    def update(self, t: float, detections: list[dict[str, Any]]) -> None:
        """Set each detection's "id"."""
        if t < self.last_time:
            self.tracks.clear()  # the clock jumped back
        self.last_time = t
        self.tracks = {
            track_id: track
            for track_id, track in self.tracks.items()
            if t - track["t"] <= self.timeout_s
        }
        pairs = []
        for track_id, track in self.tracks.items():
            for index, detection in enumerate(detections):
                if detection["class"] != track["class"]:
                    continue
                box, other = track["bbox_xywh"], detection["bbox_xywh"]
                size = max(1.0, math.hypot(box[2], box[3]), math.hypot(other[2], other[3]))
                cost = math.dist(box[:2], other[:2]) / size
                if cost <= self.gate:
                    pairs.append((cost, int(track_id), index))
        assigned: dict[int, str] = {}
        for _, track_id, index in sorted(pairs):
            if index not in assigned and str(track_id) not in assigned.values():
                assigned[index] = str(track_id)
        for index, detection in enumerate(detections):
            track_id = assigned.get(index)
            if track_id is None:
                track_id = str(self.next_id)
                self.next_id += 1
            detection["id"] = track_id
            self.tracks[track_id] = {
                "t": float(t),
                "class": detection["class"],
                "bbox_xywh": list(detection["bbox_xywh"]),
                "centroid_px": list(detection["centroid_px"]),
            }


def parse_no_fire_zones(values: Any) -> list[tuple[float, float, float, float]]:
    """no_fire_zones_rad (pan_min, pan_max, tilt_min, tilt_max, ...) as
    boxes, without the empty ones (min == max). Raises ValueError on a
    malformed list: a zone that silently stops working is worse than a
    node that does not start."""
    values = [float(value) for value in values]
    if len(values) % 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("no_fire_zones_rad needs finite values in fours: "
                         "pan_min, pan_max, tilt_min, tilt_max")
    zones = []
    for i in range(0, len(values), 4):
        pan_min, pan_max, tilt_min, tilt_max = values[i:i + 4]
        if pan_min > pan_max or tilt_min > tilt_max:
            raise ValueError(f"no_fire_zones_rad box {i // 4}: a minimum above its maximum")
        if pan_min < pan_max and tilt_min < tilt_max:
            zones.append((pan_min, pan_max, tilt_min, tilt_max))
    return zones


def in_no_fire_zone(
    zones: list[tuple[float, float, float, float]], pan: float, tilt: float,
    margin: float = 0.0,
) -> bool:
    """Whether (pan, tilt) is in a zone widened by margin on every side."""
    return any(pan_min - margin <= pan <= pan_max + margin
               and tilt_min - margin <= tilt <= tilt_max + margin
               for pan_min, pan_max, tilt_min, tilt_max in zones)


class FireCycle:
    """The dart launcher's fire cycle (no ROS dependencies): prime (the rack
    back to the sear, ``stroke_m``), chamber (the rack forward again), fire
    (the trigger to ``fire_rad``, held ``dwell_s``, then back to 0), each
    step verified against the measured rack and trigger within
    ``timeout_s``. The trigger moves only while update() is told the line
    of fire is clear: chambered with it blocked, or blocked before the sear
    released, is a fault.

    start() is the only way in: goals() asks for the rack back only while
    priming (for at most timeout_s) and for the trigger off 0 only while
    firing (for at most timeout_s plus dwell_s), however late update() runs.
    Anything else asks for the trigger at 0 and the rack forward or held
    where it is (None). A cycle ends in a shot ("fired") or a fault: a step
    not reached in time, no measured position, or an abort (the trigger to
    0, the rack stopped). Faults latch until reset(); a launcher found away
    from rest with no cycle running is returned to rest under supervision.
    """

    READY = "ready"
    PRIMING = "priming"
    CHAMBERING = "chambering"
    FIRING = "firing"  # the trigger out and held
    RESETTING = "resetting"  # the trigger back to 0 after the shot
    FAULT = "fault"
    RETURNING = "returning"  # back to rest after a fault or when found away from it

    def __init__(
        self,
        stroke_m: float = 0.038,
        fire_rad: float = 0.6,
        dwell_s: float = 0.2,
        timeout_s: float = 3.0,
        rack_tolerance_m: float = RACK_TOLERANCE_M,
        trigger_tolerance_rad: float = TRIGGER_TOLERANCE_RAD,
    ) -> None:
        if not 0.0 < rack_tolerance_m < stroke_m / 2.0:
            raise ValueError("the rack tolerance must be positive and under half the stroke")
        if not 0.0 < trigger_tolerance_rad < fire_rad / 2.0:
            raise ValueError("the trigger tolerance must be positive and under half the angle")
        if not (dwell_s >= 0.0 and timeout_s > 0.0):
            raise ValueError("dwell_s must not be negative and timeout_s must be positive")
        self.stroke = float(stroke_m)
        self.fire = float(fire_rad)
        self.dwell_s = float(dwell_s)
        self.timeout_s = float(timeout_s)
        self.rack_tolerance = float(rack_tolerance_m)
        self.trigger_tolerance = float(trigger_tolerance_rad)
        self.phase = self.READY
        self.since = -math.inf  # when the phase began
        self.released = -math.inf  # when the trigger reached the fire angle
        self.target_id = ""  # the running cycle's target, "" without one
        self.reason = ""  # the latched fault

    @property
    def running(self) -> bool:
        return bool(self.target_id)

    def goals(self, now: float) -> tuple[float | None, float]:
        """(rack, trigger) to command; rack None: hold it where it is."""
        if self.phase == self.PRIMING:
            return (self.stroke if now - self.since <= self.timeout_s else None), 0.0
        if self.phase == self.FIRING and now - self.since <= self.timeout_s + self.dwell_s:
            return 0.0, self.fire
        if self.phase == self.FAULT:
            return None, 0.0
        return 0.0, 0.0

    def start(self, now: float, target_id: str) -> None:
        """Begin a cycle on target_id. Only the tag command's handler calls
        this, after checking every gate."""
        self.phase, self.since, self.released = self.PRIMING, float(now), -math.inf
        self.target_id, self.reason = str(target_id), ""

    def abort(self, now: float, reason: str) -> tuple[str, str, str, bool] | None:
        """End a cycle in progress: before the trigger released, a fault
        (the trigger to 0, the rack stopped); after, it completes as fired.
        Returns the fault's event, if any."""
        if self.phase in (self.PRIMING, self.CHAMBERING):
            return self._fault(now, f"aborted while {self.phase} ({reason})")
        if self.phase == self.FIRING:
            if self.released > -math.inf:
                self.phase, self.since = self.RESETTING, float(now)  # the dart is gone
                return None
            return self._fault(now, f"aborted while firing ({reason}): the dart may have gone",
                               shot=True)
        return None

    def reset(self, now: float) -> bool:
        """Clear a latched fault; the launcher returns to rest under supervision."""
        if self.phase != self.FAULT:
            return False
        self.phase, self.since, self.reason = self.RETURNING, float(now), ""
        return True

    def at_rest(self, rack: float | None, trigger: float | None) -> bool:
        return (rack is not None and trigger is not None and rack <= self.rack_tolerance
                and trigger <= self.trigger_tolerance)

    def update(
        self, now: float, rack: float | None, trigger: float | None, clear: bool = True
    ) -> tuple[str, str, str, bool] | None:
        """Advance on the measured rack and trigger (None: no fresh
        measurement) and whether the line of fire is clear now. Returns
        (target_id, result, reason, shot) when a cycle ends ("fired" or
        "fault") or a fault latches (target_id "")."""
        phase = self.phase
        if phase in (self.READY, self.FAULT):
            if phase == self.READY and (
                (rack is not None and rack > 2.0 * self.rack_tolerance)
                or (trigger is not None and trigger > 2.0 * self.trigger_tolerance)
            ):
                # Away from rest with no cycle (moved by hand, or left so at
                # start-up): back to rest, verified as after a fault.
                self.phase, self.since = self.RETURNING, float(now)
            return None
        if rack is None or trigger is None:
            return self._fault(now, "no launcher position", shot=self.released > -math.inf)
        late = now - self.since > self.timeout_s
        if phase == self.PRIMING:
            if rack >= self.stroke - self.rack_tolerance:
                self.phase, self.since = self.CHAMBERING, float(now)
            elif late:
                return self._fault(now, f"priming not reached in {self.timeout_s:g} s")
        elif phase == self.CHAMBERING:
            if rack <= self.rack_tolerance:
                if not clear:
                    # The moment of firing, and the dart would hit the vehicle
                    return self._fault(now, "line of fire blocked")
                self.phase, self.since = self.FIRING, float(now)
            elif late:
                return self._fault(now, f"chambering not reached in {self.timeout_s:g} s")
        elif phase == self.FIRING:
            if self.released == -math.inf and not clear:
                return self._fault(now, "line of fire blocked while firing: the dart may have gone",
                                   shot=True)
            if self.released == -math.inf and trigger >= self.fire - self.trigger_tolerance:
                self.released = float(now)
            if self.released > -math.inf:
                if now - self.released >= self.dwell_s:
                    self.phase, self.since = self.RESETTING, float(now)
            elif late:
                return self._fault(now, f"trigger not reached in {self.timeout_s:g} s")
        elif phase == self.RESETTING:
            if trigger <= self.trigger_tolerance:
                event = (self.target_id, "fired", "", True)
                self.phase, self.target_id = self.READY, ""
                return event
            if late:
                return self._fault(now, f"trigger not back at 0 in {self.timeout_s:g} s",
                                   shot=True)
        elif phase == self.RETURNING:
            if self.at_rest(rack, trigger):
                self.phase = self.READY
            elif late:
                return self._fault(now, f"not back at rest in {self.timeout_s:g} s")
        return None

    def _fault(self, now: float, reason: str, shot: bool = False) -> tuple[str, str, str, bool]:
        event = (self.target_id, "fault", reason, shot)
        self.phase, self.since = self.FAULT, float(now)
        self.target_id, self.reason = "", reason
        return event


class PanTiltController:
    """Latency-compensated pan/tilt position controller (no ROS dependencies).

    A camera frame captured at time ``t`` reports the target's angular offset
    ``e`` from the optical axis. The target direction in joint space is
    ``theta = q(t) + e``, where ``q(t)`` is where the turret pointed when the
    frame was captured: measured joint states interpolated at ``t`` when the
    actuator reports them, otherwise the command in effect at
    ``t - actuator_latency_s``. Each frame moves the command a fraction
    ``gain`` of the way from the previous command toward ``theta``, at most
    ``max_rate_rad_s``.

    With the position at capture time known, ``theta`` does not depend on the
    turret's own motion, so the loop is a first-order filter (pole
    ``1 - gain``) whatever the camera/inference/actuator latency. If that
    compensation were lost entirely, the update would degrade to integrating
    the image error with an n-frame delay, which is stable only for
    ``gain < 2 sin(pi / (2 (2 n + 1)))`` = 1.0, 0.62, 0.45, 0.35 for
    n = 1..4 frames; the 0.82 per-frame gain this replaces is unstable from two
    frames of delay (pole magnitude 1.09). Keep ``gain`` at or below 0.45 when
    the latency estimate is uncertain.

    On a moving base (``attitude``: the base's rotation into a world-fixed
    frame, from its gyro) or with a target whose bearing keeps changing
    (``lead``: its line-of-sight rate in that frame), the same filter runs on
    world directions instead of joint angles: the target's line of sight is
    predicted to when the command will take effect (``now`` plus
    :meth:`actuation_delay`), the previous aim is carried along by the base's
    rotation and the lead, and both are turned back into joint angles through
    the base's attitude at that time. A target that moves as predicted then
    leaves no lag, and :meth:`hold` keeps the aim on it between frames.
    Without either, the update is the fixed-base one above.
    """

    def __init__(
        self,
        gain: float = 0.4,
        max_rate_rad_s: float = 1.5,
        pan_limits: tuple[float, float] = (-1.5708, 1.5708),
        tilt_limits: tuple[float, float] = (-0.65, 0.75),
        actuator_latency_s: float = 0.1,
        feedback_timeout_s: float = 0.5,
        resume_timeout_s: float = 0.5,
        history_s: float = 5.0,
        initial_position: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        if not 0.0 < gain <= 1.0:
            raise ValueError("gain must be in (0, 1]")
        if not max_rate_rad_s > 0.0:
            raise ValueError("max_rate_rad_s must be positive")
        if actuator_latency_s < 0.0:
            raise ValueError("actuator_latency_s must not be negative")
        self.gain = float(gain)
        self.max_rate = float(max_rate_rad_s)
        self.limits = (
            (float(pan_limits[0]), float(pan_limits[1])),
            (float(tilt_limits[0]), float(tilt_limits[1])),
        )
        self.actuator_latency_s = float(actuator_latency_s)
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.resume_timeout_s = float(resume_timeout_s)
        self.history_s = float(history_s)
        self.measured: deque[tuple[float, float, float]] = deque()
        # Until a command is sent the turret is assumed to be at its start pose.
        self.commanded: deque[tuple[float, float, float]] = deque(
            [(-math.inf, float(initial_position[0]), float(initial_position[1]))]
        )
        self.last_command: tuple[float, float, float] | None = None
        self.last_update: float | None = None
        # Moving-base aim: (time it applies, world direction, line-of-sight rate).
        self.aim: tuple[float, np.ndarray, np.ndarray] | None = None
        self.delay = self.actuator_latency_s
        self.delay_time = -math.inf

    def add_measured(self, t: float, pan: float, tilt: float) -> bool:
        if not finite(t, pan, tilt):
            return False
        if self.measured and t < self.measured[-1][0]:
            self.measured.clear()  # source restarted or the clock jumped back
        self.measured.append((float(t), float(pan), float(tilt)))
        self._prune(self.measured, t)
        return True

    def has_feedback(self, now: float) -> bool:
        return bool(self.measured) and now - self.measured[-1][0] <= self.feedback_timeout_s

    def continuing(self, now: float) -> bool:
        """Whether the last command is recent enough to continue from."""
        return (
            self.last_command is not None
            and 0.0 <= now - self.last_command[0] <= self.resume_timeout_s
        )

    def position(self, now: float) -> tuple[float, float, float]:
        """(time, pan, tilt): the best estimate of where the turret is going."""
        if self.continuing(now):
            return self.last_command
        # First command, or resuming after a pause: start from the best
        # estimate of where the turret is now rather than a stale command.
        return self.measured[-1] if self.has_feedback(now) else self.commanded[-1]

    def actuation_delay(self, now: float) -> float:
        """Delay from a command to the turret's motion, the horizon the
        moving-base aim predicts over: fitted to the measured joint states
        (twice a second, while the commands move the turret), otherwise
        ``actuator_latency_s``. A wrong horizon points the turret ahead of or
        behind the target by the base's rate times the error, which the
        camera loop cannot see as its own."""
        if not self.has_feedback(now):
            return self.actuator_latency_s
        if not 0.0 <= now - self.delay_time < 0.5:
            self.delay_time = now
            fitted = self._fit_delay(now)
            if fitted is not None:
                self.delay = 0.5 * (self.delay + fitted)
        return self.delay

    def _fit_delay(
        self, now: float, window_s: float = 2.0, max_delay_s: float = 0.3
    ) -> float | None:
        """The delay that best explains the last ``window_s`` of measured
        positions as the commands in effect that much earlier; None while the
        commands barely moved (a still turret says nothing about its delay)."""
        commanded = np.array(self.commanded)
        # Only positions every candidate delay maps to a sent command (not the
        # assumed start pose before the first one).
        start = max(now - window_s, commanded[min(1, len(commanded) - 1), 0] + max_delay_s)
        measured = np.array([sample for sample in self.measured if sample[0] >= start])
        recent = commanded[commanded[:, 0] >= start - max_delay_s, 1:]
        if len(measured) < 10 or len(recent) < 2 or np.ptp(recent, axis=0).max() < 0.05:
            return None
        delays = np.arange(0.0, max_delay_s + 1e-9, 0.01)
        costs = []
        for delay in delays:
            index = np.searchsorted(commanded[:, 0], measured[:, 0] - delay, side="right")
            residual = measured[:, 1:] - commanded[np.maximum(index - 1, 0), 1:]
            costs.append(float(np.mean(residual**2)))
        return float(delays[int(np.argmin(costs))])

    def reference(self, t_image: float, now: float) -> tuple[tuple[float, float], str]:
        """Turret position when the frame stamped ``t_image`` was captured."""
        if self.has_feedback(now):
            return self._lookup(self.measured, t_image, True), "measured"
        return (
            self._lookup(self.commanded, t_image - self.actuator_latency_s, False),
            "commanded",
        )

    def sight(
        self,
        t_image: float,
        err_pan: float,
        err_tilt: float,
        now: float,
        attitude: Attitude | None = None,
    ) -> np.ndarray:
        """The target's line of sight at capture, in the world-fixed frame of
        ``attitude`` (turret/base_link without one)."""
        (pan, tilt), _ = self.reference(t_image, now)
        rotate = attitude or fixed_attitude
        return rotate(t_image) @ camera_ray(pan, tilt, err_pan, err_tilt)

    def update(
        self,
        t_image: float,
        err_pan: float,
        err_tilt: float,
        now: float,
        attitude: Attitude | None = None,
        lead: np.ndarray | None = None,
    ) -> tuple[float, float] | None:
        """Return the next (pan, tilt) command, or None for invalid input."""
        if not finite(t_image, err_pan, err_tilt, now):
            return None
        if lead is not None and not finite(*lead):
            lead = None
        dt = self._step_time(now)
        if attitude is None and lead is None:
            self.aim = None
            reference, _ = self.reference(t_image, now)
            target = (reference[0] + err_pan, reference[1] + err_tilt)
            return self._command(now, self.position(now)[1:], target, dt, False)

        rotate = attitude or fixed_attitude
        rate = np.zeros(3) if lead is None else np.asarray(lead, dtype=float)
        t_aim = now + self.actuation_delay(now)
        to_base = rotate(t_aim).T
        sight = self.sight(t_image, err_pan, err_tilt, now, attitude)
        target = direction_joints(to_base @ unit(sight + rate * (t_aim - t_image)))
        if self.continuing(now) and self.aim is not None:
            previous = self.hold(t_aim, attitude)
        else:
            t_q, pan, tilt = self.position(now)
            previous = direction_joints(to_base @ rotate(t_q) @ joint_direction(pan, tilt))
        command = self._command(now, previous, target, dt, True)
        self.aim = (t_aim, rotate(t_aim) @ joint_direction(*command), rate)
        return command

    def hold(
        self, t: float, attitude: Attitude | None = None
    ) -> tuple[float, float] | None:
        """Joints that keep the moving-base aim on its world direction at time
        ``t`` (when the command takes effect), extrapolating the lead for at
        most ``resume_timeout_s`` past the last frame."""
        if self.aim is None or not finite(t):
            return None
        t_aim, direction, rate = self.aim
        elapsed = min(max(t - t_aim, 0.0), self.resume_timeout_s)
        rotate = attitude or fixed_attitude
        pan, tilt = direction_joints(rotate(t).T @ unit(direction + rate * elapsed))
        return clamp(pan, *self.limits[0]), clamp(tilt, *self.limits[1])

    def record(self, now: float, pan: float, tilt: float) -> None:
        """Note a command sent to the actuator."""
        self.last_command = (float(now), float(pan), float(tilt))
        self.commanded.append(self.last_command)
        self._prune(self.commanded, now)

    def _step_time(self, now: float) -> float:
        # The rate limit applies per camera update, so it uses the time since
        # the last update even when hold() commands were sent in between.
        if (
            self.continuing(now)
            and self.last_update is not None
            and 0.0 <= now - self.last_update <= self.resume_timeout_s
        ):
            return max(1e-3, now - self.last_update)
        return min(self.resume_timeout_s, 0.1)

    def _command(
        self,
        now: float,
        previous: tuple[float, float],
        target: tuple[float, float],
        dt: float,
        wrap: bool,
    ) -> tuple[float, float]:
        step_limit = self.max_rate * dt
        command = []
        for prev, goal, (low, high) in zip(previous, target, self.limits):
            error = math.remainder(goal - prev, 2.0 * math.pi) if wrap else goal - prev
            step = max(-step_limit, min(step_limit, self.gain * error))
            command.append(clamp(prev + step, low, high))
        self.record(now, command[0], command[1])
        self.last_update = float(now)
        return command[0], command[1]

    def _prune(self, history: deque, now: float) -> None:
        while len(history) > 2 and now - history[1][0] > self.history_s:
            history.popleft()

    @staticmethod
    def _lookup(history: deque, t: float, interpolate: bool) -> tuple[float, float]:
        times = [sample[0] for sample in history]
        index = bisect_right(times, t)
        if index == 0:
            return history[0][1:]
        if index == len(history) or not interpolate:
            return history[index - 1][1:]
        (t0, *first), (t1, *second) = history[index - 1], history[index]
        ratio = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        return tuple(a + (b - a) * ratio for a, b in zip(first, second))


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert common ROS Image encodings without requiring cv_bridge."""
    encodings = {
        "bgr8": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
    }
    if msg.encoding not in encodings:
        raise ValueError(f"unsupported camera encoding: {msg.encoding}")
    channels, conversion = encodings[msg.encoding]
    minimum_step = int(msg.width) * channels
    if int(msg.step) < minimum_step:
        raise ValueError(
            f"invalid image step {msg.step} for {msg.width}x{msg.encoding}"
        )
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        raise ValueError(f"short image buffer: {raw.size} < {expected}")
    rows = raw[:expected].reshape(int(msg.height), int(msg.step))
    pixels = rows[:, :minimum_step]
    if channels == 1:
        image = pixels.reshape(int(msg.height), int(msg.width))
    else:
        image = pixels.reshape(int(msg.height), int(msg.width), channels)
    image = np.ascontiguousarray(image)
    return cv2.cvtColor(image, conversion) if conversion is not None else image


def mask_message(mask: np.ndarray, source: Image) -> Image:
    msg = Image()
    msg.header = source.header
    msg.height = int(mask.shape[0])
    msg.width = int(mask.shape[1])
    msg.encoding = "mono8"
    msg.is_bigendian = False
    msg.step = int(mask.shape[1])
    msg.data = np.ascontiguousarray(mask, dtype=np.uint8).tobytes()
    return msg


def bgr_message(frame: np.ndarray, source: Image) -> Image:
    """Create a ROS bgr8 image while preserving the source timestamp/frame."""
    msg = Image()
    msg.header = source.header
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(frame.shape[1]) * 3
    msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
    return msg


def joint_position(msg: JointState | None, name: str) -> float | None:
    if msg is not None and name in msg.name:
        index = msg.name.index(name)
        if index < len(msg.position):
            return float(msg.position[index])
    return None


COMMANDING = ("measured", "commanded")

DEFAULT_PARAMETERS: dict[str, Any] = {
    "camera_topic": "/turret/camera/image_raw",
    "camera_info_topic": "/turret/camera/camera_info",
    "joint_state_topic": "/turret/joint_states",
    "joint_command_topic": "/turret/joint_commands",
    "segments_topic": "/turret/yolo/segments",
    "target_mask_topic": "/turret/yolo/target_mask",
    "target_bbox_topic": "/turret/yolo/target_bbox",
    "target_confidence_topic": "/turret/yolo/target_confidence",
    "pixel_error_topic": "/turret/target/pixel_error",
    "angular_error_topic": "/turret/target/angular_error",
    "annotated_image_topic": "/turret/yolo/annotated_image",
    "selected_id_topic": "/turret/target/selected_id",
    "locked_topic": "/turret/target/locked",
    "diagnostics_topic": "/turret/diagnostics",
    "emergency_stop_topic": "/turret/emergency_stop",
    "actuator_ready_topic": "/turret/actuator_ready",
    "command_topic": "/turret/command",
    "status_topic": "/turret/status",
    "model_path": "/workspace/cache/models/yoloe-26s-seg.pt",
    "device": "0",
    "image_size": 640,
    "confidence": 0.003,
    "class_prompts": ["blue utility cart"],
    "target_id": "cart_blue_01",
    "target_class": "blue utility cart",
    "initial_selection_normalized": [0.13, 0.565],
    "association_gate_fraction": 0.50,
    # Track target_class from start-up (the selection above); false waits
    # for a lock command on command_topic.
    "auto_start": True,
    # Detections keep their ID this long without being seen.
    "track_timeout_s": 1.0,
    "command_enabled": True,
    # Commands stay disabled until a false emergency-stop message arrives.
    "require_estop_release": False,
    # Fraction of the remaining pointing error corrected per frame; see
    # PanTiltController for the latency/stability trade-off.
    "kp": 0.4,
    "max_rate_rad_s": 1.5,
    # Command-to-motion delay used when no measured joint states arrive.
    "actuator_latency_s": 0.1,
    "joint_state_timeout_s": 0.5,
    "actuator_ready_timeout_s": 1.0,
    "pan_min_rad": -1.5708,
    "pan_max_rad": 1.5708,
    "tilt_min_rad": -0.65,
    "tilt_max_rad": 0.75,
    # Moving base: the base's angular velocity (sensor_msgs/Imu), rotated into
    # base_frame through TF from the messages' frame_id (or base_imu_frame).
    "base_compensation": True,
    "base_imu_topic": "/imu/data_raw",
    "base_imu_frame": "",
    "base_frame": "turret/base_link",
    "imu_timeout_s": 0.5,
    # Feed forward the target's bearing rate, estimated over lead_window_s.
    "target_lead": True,
    "lead_window_s": 0.5,
    # Manual rates, centring and the moving-base aim between frames.
    "control_rate_hz": 50.0,
    "manual_timeout_s": 0.5,
    # Wall-clock rate, so the UI sees the status however fast sim time runs.
    "status_rate_hz": 5.0,
    "lock_tolerance_px": 36.0,
    # Foam dart launcher (systems/racecar_turret/interfaces.md): its priming
    # rack (actuator_joint, m) and trigger (trigger_joint, rad) join the joint
    # commands and states. A {"action": "tag"} command naming the locked
    # target fires one dart (prime to launcher_stroke_m, chamber, trigger to
    # trigger_fire_rad held tag_dwell_s, trigger back) only while it is locked
    # within tag_max_aim_error_deg at the aim point, armed, the actuator
    # ready, the launcher at rest with darts, and tag_min_interval_s after
    # the last shot.
    "actuator_fitted": False,
    "launcher_stroke_m": 0.038,
    "trigger_fire_rad": 0.6,
    "tag_dwell_s": 0.2,
    "tag_max_aim_error_deg": 2.0,
    "tag_min_interval_s": 2.0,
    # A step not measured within this long faults (the trigger to 0, the
    # rack stopped; latched until the emergency stop is engaged and released).
    "tag_stroke_timeout_s": 3.0,
    # A cycle stops when its target has not been seen locked this long.
    "tag_lock_timeout_s": 0.5,
    # After a shot, aim at (lock) the nearest untagged detection; it never fires.
    "auto_next": True,
    # Darts in the clip at start-up; {"action": "reload", "darts": N} sets it.
    "darts": 0,
    # With the launcher, a person is aimed at (and its aim error measured at)
    # the torso: person_aim_fraction of its box's height from the top; a box
    # taller than person_max_box_fraction of the image is too close to fire.
    "person_aim_fraction": 0.6,
    "person_max_box_fraction": 0.6,
    # The dart's drop: an aim offset (pitch up, deg) against the range (m),
    # interpolated, the range estimated from a person's box height and
    # person_height_m. The default table is all zero: no offset.
    "person_height_m": 1.7,
    "aim_offset_range_m": [1.0, 10.0],
    "aim_offset_pitch_deg": [0.0, 0.0],
    # Line of fire: boxes of joint positions where a dart would hit the
    # vehicle, four values each: pan_min, pan_max, tilt_min, tilt_max (rad),
    # widened by no_fire_margin_rad (the turret keeps moving during the
    # trigger's pull). Firing is refused while the measured or the commanded
    # pan and tilt are in one: before priming, and from the moment of firing
    # until the sear releases. A box with min == max is empty: the default
    # is none.
    "no_fire_zones_rad": [0.0, 0.0, 0.0, 0.0],
    "no_fire_margin_rad": 0.05,
    "publish_annotated_image": True,
    "show_window": False,
    "window_name": "PanTiltROS YOLO",
    "warmup": True,
    "ready_file": "/tmp/pantilt_yolo_ready",
}


class PanTiltYoloTracker(Node):
    def __init__(self) -> None:
        super().__init__("pantilt_yolo_tracker")
        for name, default in DEFAULT_PARAMETERS.items():
            self.declare_parameter(name, default)
        self._configure()

        self.segments_pub = self.create_publisher(
            String, str(self.parameter("segments_topic")), 10
        )
        self.mask_pub = self.create_publisher(
            Image, str(self.parameter("target_mask_topic")), 10
        )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, str(self.parameter("target_bbox_topic")), 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, str(self.parameter("target_confidence_topic")), 10
        )
        self.pixel_error_pub = self.create_publisher(
            Float32MultiArray, str(self.parameter("pixel_error_topic")), 10
        )
        self.angular_error_pub = self.create_publisher(
            Float32MultiArray, str(self.parameter("angular_error_topic")), 10
        )
        self.annotated_pub = self.create_publisher(
            Image,
            str(self.parameter("annotated_image_topic")),
            qos_profile_sensor_data,
        )
        self.selected_id_pub = self.create_publisher(
            String, str(self.parameter("selected_id_topic")), 10
        )
        self.locked_pub = self.create_publisher(
            Bool, str(self.parameter("locked_topic")), 10
        )
        self.command_pub = self.create_publisher(
            JointState, str(self.parameter("joint_command_topic")), 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.parameter("diagnostics_topic")), 10
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status_pub = self.create_publisher(
            String, str(self.parameter("status_topic")), latched_qos
        )
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CameraInfo,
            str(self.parameter("camera_info_topic")),
            self._camera_info,
            camera_qos,
        )
        self.create_subscription(
            JointState,
            str(self.parameter("joint_state_topic")),
            self._joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.parameter("camera_topic")),
            self._image,
            camera_qos,
        )
        # The latched subscription receives the last state from transient-local
        # publishers; the volatile one also matches default-QoS publishers
        # (e.g. a plain `ros2 topic pub`), which a transient-local reader cannot.
        for qos in (latched_qos, 10):
            self.create_subscription(
                Bool,
                str(self.parameter("emergency_stop_topic")),
                self._emergency_stop,
                qos,
            )
        self.create_subscription(
            Bool,
            str(self.parameter("actuator_ready_topic")),
            self._actuator_ready,
            latched_qos,
        )
        self.create_subscription(
            String, str(self.parameter("command_topic")), self._turret_command, 10
        )
        if self.base_compensation:
            if tf2_ros is not None:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            # Inference blocks this executor for tens of milliseconds; keep
            # the rates that arrive meanwhile.
            imu_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
            )
            self.create_subscription(
                Imu, str(self.parameter("base_imu_topic")), self._imu, imu_qos
            )
        # Node (simulation) time: the deadman, manual rates and the aim follow
        # the clock the images and joint states are stamped with.
        self.create_timer(self.control_period_s, self._control_tick)
        self.create_timer(
            1.0 / max(0.5, float(self.parameter("status_rate_hz"))),
            self._publish_status,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
        )
        self._publish_status()

        self.get_logger().info(
            f"Loading Ultralytics checkpoint {self.model_path} on {self.device}"
        )
        self.model = YOLO(self.model_path)
        if "yoloe-" in Path(self.model_path).name.lower():
            self.model.set_classes(self.class_prompts)
        if bool(self.parameter("warmup")):
            self.model.predict(
                np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8),
                conf=self.confidence,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )
        self.ready_file.parent.mkdir(parents=True, exist_ok=True)
        self.ready_file.write_text("ready\n")
        self.get_logger().info(
            f"PanTiltROS YOLO ready: target={self.target_class!r}, "
            f"camera={self.parameter('camera_topic')}"
        )

    def _configure(self) -> None:
        """Read the parameters and set up the tracking state (no ROS entities)."""
        self.model_path = str(self.parameter("model_path"))
        self.device = str(self.parameter("device"))
        self.image_size = int(self.parameter("image_size"))
        self.confidence = float(self.parameter("confidence"))
        self.class_prompts = [str(value) for value in self.parameter("class_prompts")]
        self.target_id = str(self.parameter("target_id"))
        self.target_class = str(self.parameter("target_class"))
        initial = list(self.parameter("initial_selection_normalized"))
        if len(initial) != 2:
            raise ValueError("initial_selection_normalized must contain x and y")
        self.initial_selection = (float(initial[0]), float(initial[1]))
        self.association_gate_fraction = float(
            self.parameter("association_gate_fraction")
        )
        self.command_enabled = bool(self.parameter("command_enabled"))
        self.pan_min = float(self.parameter("pan_min_rad"))
        self.pan_max = float(self.parameter("pan_max_rad"))
        self.tilt_min = float(self.parameter("tilt_min_rad"))
        self.tilt_max = float(self.parameter("tilt_max_rad"))
        self.joint_state_timeout_s = float(self.parameter("joint_state_timeout_s"))
        self.controller = PanTiltController(
            gain=float(self.parameter("kp")),
            max_rate_rad_s=float(self.parameter("max_rate_rad_s")),
            pan_limits=(self.pan_min, self.pan_max),
            tilt_limits=(self.tilt_min, self.tilt_max),
            actuator_latency_s=float(self.parameter("actuator_latency_s")),
            feedback_timeout_s=self.joint_state_timeout_s,
        )
        self.estop_engaged = bool(self.parameter("require_estop_release"))
        self.actuator_ready = False
        self.actuator_ready_time = -math.inf
        self.actuator_ready_timeout_s = float(
            self.parameter("actuator_ready_timeout_s")
        )
        self.command_state = "idle"
        self.warned_stamp = False
        self.lock_tolerance = float(self.parameter("lock_tolerance_px"))
        self.publish_annotated_image = bool(
            self.parameter("publish_annotated_image")
        )
        self.show_window = bool(self.parameter("show_window"))
        self.window_name = str(self.parameter("window_name"))
        self.window_initialized = False
        if self.show_window and not os.environ.get("DISPLAY"):
            self.get_logger().warning(
                "show_window is enabled but DISPLAY is unset; disabling the window"
            )
            self.show_window = False
        self.ready_file = Path(str(self.parameter("ready_file")))

        self.camera_info: CameraInfo | None = None
        self.target_centroid: tuple[float, float] | None = None
        self.frame_count = 0

        # Selection: "startup" follows target_class from initial_selection as
        # configured; "track" follows the detection ID chosen by a command.
        self.mode = "auto" if bool(self.parameter("auto_start")) else "idle"
        self.selection = "startup" if self.mode == "auto" else ""
        self.target_track = ""
        self.lock_class = ""
        self.tracks = DetectionTracks(timeout_s=float(self.parameter("track_timeout_s")))
        self.last_detections: list[dict[str, Any]] = []
        self.last_frame: dict[str, Any] | None = None
        self.manual_rates = (0.0, 0.0)
        self.manual_time = -math.inf
        self.manual_timeout_s = float(self.parameter("manual_timeout_s"))
        self.manual_last: tuple[tuple[float, float], float] | None = None
        self.goto: tuple[float, float] | None = None
        self.notice = ("", -math.inf)
        self.control_period_s = 1.0 / max(1.0, float(self.parameter("control_rate_hz")))

        self.base_compensation = bool(self.parameter("base_compensation"))
        self.base_imu_topic = str(self.parameter("base_imu_topic"))
        self.base_imu_frame = str(self.parameter("base_imu_frame"))
        self.base_frame = str(self.parameter("base_frame"))
        self.imu_timeout_s = float(self.parameter("imu_timeout_s"))
        self.base_rotation = BaseRotation(timeout_s=self.imu_timeout_s)
        self.tf_buffer: Any = None
        self.imu_rotations: dict[str, np.ndarray] = {}
        self.imu_lookup_time = -math.inf
        self.imu_problem = ""
        self.target_lead = bool(self.parameter("target_lead"))
        self.lead = BearingRate(
            window_s=float(self.parameter("lead_window_s")),
            max_rate_rad_s=self.controller.max_rate,
        )
        self.lead_rate: np.ndarray | None = None
        self.lead_world = False

        self.actuator_fitted = bool(self.parameter("actuator_fitted"))
        self.tag_max_aim_error_deg = float(self.parameter("tag_max_aim_error_deg"))
        self.tag_min_interval_s = float(self.parameter("tag_min_interval_s"))
        self.tag_lock_timeout_s = float(self.parameter("tag_lock_timeout_s"))
        self.auto_next = bool(self.parameter("auto_next"))
        self.fire_cycle = FireCycle(
            stroke_m=float(self.parameter("launcher_stroke_m")),
            fire_rad=float(self.parameter("trigger_fire_rad")),
            dwell_s=float(self.parameter("tag_dwell_s")),
            timeout_s=float(self.parameter("tag_stroke_timeout_s")),
        )
        self.darts = max(0, int(self.parameter("darts")))
        self.person_aim_fraction = float(self.parameter("person_aim_fraction"))
        self.person_max_box_fraction = float(self.parameter("person_max_box_fraction"))
        self.person_height_m = float(self.parameter("person_height_m"))
        ranges = [float(value) for value in self.parameter("aim_offset_range_m")]
        pitches = [float(value) for value in self.parameter("aim_offset_pitch_deg")]
        if len(ranges) != len(pitches) or not ranges or any(
            b <= a for a, b in zip(ranges, ranges[1:])
        ):
            raise ValueError(
                "aim_offset_range_m (increasing) and aim_offset_pitch_deg need the same length"
            )
        self.aim_offset_table = (ranges, pitches)
        self.no_fire_zones = parse_no_fire_zones(self.parameter("no_fire_zones_rad"))
        self.no_fire_margin = float(self.parameter("no_fire_margin_rad"))
        if not (math.isfinite(self.no_fire_margin) and self.no_fire_margin >= 0.0):
            raise ValueError("no_fire_margin_rad must be 0 or more")
        # Measured launcher: (stamp, rack metres, trigger radians)
        self.launcher_measured: tuple[float, float, float] | None = None
        self.shots = 0
        self.tagged_ids: list[str] = []
        self.last_tag: dict[str, Any] | None = None
        self.auto_next_pending = False
        self.last_cycle_end = -math.inf
        self.tag_seen_time = -math.inf  # last frame with the cycle's target locked
        self.last_send_time = -math.inf

    def parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _joint_state(self, msg: JointState) -> None:
        pan = joint_position(msg, "pan_joint")
        tilt = joint_position(msg, "tilt_joint")
        if pan is None or tilt is None:
            return
        stamp = stamp_seconds(msg.header.stamp)
        now = self._now()
        if now <= 0.0:
            return  # use_sim_time and no /clock yet
        if stamp <= 0.0 or abs(stamp - now) > 2.0:
            stamp = now
        if self.actuator_fitted:
            rack = joint_position(msg, ACTUATOR_JOINT)
            trigger = joint_position(msg, TRIGGER_JOINT)
            if rack is not None and trigger is not None and finite(rack, trigger):
                self.launcher_measured = (stamp, rack, trigger)
        if not self.controller.add_measured(stamp, pan, tilt):
            self.get_logger().warning(
                "ignoring non-finite joint state", throttle_duration_sec=5.0
            )

    def _imu(self, msg: Imu) -> None:
        now = self._now()
        if now <= 0.0:
            return
        rotation = self._imu_rotation(self.base_imu_frame or msg.header.frame_id, now)
        if rotation is None:
            return
        rate = msg.angular_velocity
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0 or abs(stamp - now) > 2.0:
            stamp = now
        if not self.base_rotation.add(stamp, rotation @ np.array([rate.x, rate.y, rate.z])):
            self.get_logger().warning(
                "ignoring non-finite or implausible IMU rate", throttle_duration_sec=5.0
            )

    def _imu_rotation(self, frame: str, now: float) -> np.ndarray | None:
        """Rotation from the IMU's frame into base_frame (a static transform)."""
        if frame == self.base_frame:
            return IDENTITY
        if not frame:
            self.imu_problem = "IMU messages have no frame_id: set base_imu_frame"
            return None
        if frame in self.imu_rotations:
            return self.imu_rotations[frame]
        if 0.0 <= now - self.imu_lookup_time < 1.0:
            return None
        self.imu_lookup_time = now
        if self.tf_buffer is None:
            self.imu_problem = "tf2_ros is not available"
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time()
            )
        except Exception:  # tf2's lookup, connectivity and extrapolation errors
            self.imu_problem = f"no transform from {frame} to {self.base_frame}"
            return None
        q = transform.transform.rotation
        self.imu_rotations[frame] = quaternion_matrix(q.x, q.y, q.z, q.w)
        self.imu_problem = ""
        return self.imu_rotations[frame]

    def _attitude(self) -> Attitude | None:
        """The base's attitude, or None for the fixed-base controller. Once
        rates have arrived it freezes when they stop (fixed-base aiming)
        without changing frames under the aim."""
        if self.base_compensation and self.base_rotation.samples:
            return self.base_rotation.attitude
        return None

    def _compensating(self, now: float) -> bool:
        return self.base_compensation and self.base_rotation.fresh(now)

    def _emergency_stop(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        if engaged != self.estop_engaged:
            self.get_logger().warning(
                "emergency stop " + ("engaged" if engaged else "released")
            )
        released = self.estop_engaged and not engaged
        self.estop_engaged = engaged
        if self.actuator_fitted:
            now = self._now()
            if engaged:
                self._abort_tag(now, "emergency stop")
            elif released and self.fire_cycle.reset(now):
                # Disarming and re-arming acknowledges a launcher fault.
                self._notify(now, "launcher fault reset: returning it to rest")

    def _actuator_ready(self, msg: Bool) -> None:
        self.actuator_ready = bool(msg.data)
        self.actuator_ready_time = self._now()

    def _intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
        if self.camera_info is not None and len(self.camera_info.k) == 9:
            fx = float(self.camera_info.k[0])
            fy = float(self.camera_info.k[4])
            cx = float(self.camera_info.k[2])
            cy = float(self.camera_info.k[5])
            if fx > 0.0 and fy > 0.0:
                return fx, fy, cx, cy
        return float(width), float(width), width * 0.5, height * 0.5

    def _image(self, msg: Image) -> None:
        received = self._now()
        capture_time = stamp_seconds(msg.header.stamp)
        if capture_time <= 0.0 or abs(capture_time - received) > 2.0:
            # Unusable camera clock: fall back to the receive time, which only
            # loses the transport delay from the latency compensation.
            if not self.warned_stamp:
                self.get_logger().warning(
                    "camera stamps are not on this host's clock; using receive time"
                )
                self.warned_stamp = True
            capture_time = received
        try:
            frame = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        height, width = frame.shape[:2]

        started = time.perf_counter()
        result = self.model.predict(
            frame,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )[0]
        latency_ms = (time.perf_counter() - started) * 1000.0
        detections = self._detections(result, width, height)
        self._process_frame(msg, frame, detections, latency_ms, capture_time)
        self.frame_count += 1

    def _process_frame(
        self,
        source: Image,
        frame: np.ndarray | None,
        detections: list[dict[str, Any]],
        latency_ms: float,
        capture_time: float,
    ) -> None:
        height, width = int(source.height), int(source.width)
        if self.target_centroid is None:
            self.target_centroid = (
                self.initial_selection[0] * width,
                self.initial_selection[1] * height,
            )
        self.tracks.update(capture_time, detections)
        self.last_detections = detections
        target = self._associate_target(detections, width, height)
        self._publish_outputs(
            source, frame, target, detections, latency_ms, width, height, capture_time
        )

    def _detections(self, result: Any, width: int, height: int) -> list[dict[str, Any]]:
        if result.boxes is None:
            return []
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        confidences = result.boxes.conf.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        raw_masks = None if result.masks is None else result.masks.data.detach().cpu().numpy()
        detections: list[dict[str, Any]] = []
        for index, (xyxy, confidence, class_index) in enumerate(
            zip(boxes, confidences, classes)
        ):
            x1, y1, x2, y2 = [float(value) for value in xyxy]
            mask = np.zeros((height, width), dtype=np.uint8)
            if raw_masks is not None and index < len(raw_masks):
                resized = cv2.resize(
                    raw_masks[index].astype(np.float32),
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                )
                mask[resized >= 0.5] = 255
            else:
                cv2.rectangle(
                    mask,
                    (max(0, int(x1)), max(0, int(y1))),
                    (min(width - 1, int(x2)), min(height - 1, int(y2))),
                    255,
                    -1,
                )
            ys, xs = np.nonzero(mask)
            centroid = (
                float(xs.mean()) if len(xs) else (x1 + x2) * 0.5,
                float(ys.mean()) if len(ys) else (y1 + y2) * 0.5,
            )
            detections.append(
                {
                    "id": "",  # set by DetectionTracks
                    "class": str(result.names[int(class_index)]),
                    "confidence": float(confidence),
                    # Box centre x, y and size, in pixels
                    "bbox_xywh": [
                        (x1 + x2) * 0.5,
                        (y1 + y2) * 0.5,
                        x2 - x1,
                        y2 - y1,
                    ],
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "centroid_px": [centroid[0], centroid[1]],
                    "mask_area_px": int((mask > 0).sum()),
                    "mask": mask,
                }
            )
        return detections

    def _associate_target(
        self, detections: list[dict[str, Any]], width: int, height: int
    ) -> dict[str, Any] | None:
        if self.mode != "auto":
            return None
        target = None
        if self.selection == "track":
            target = next(
                (d for d in detections if d["id"] == self.target_track), None
            )
        if target is None:
            # Start-up selection, or the locked track was lost: the detection
            # of the class nearest the last target position.
            classes = (
                (self.lock_class,)
                if self.selection == "track"
                else (self.target_class, self.target_id)
            )
            candidates = [
                detection for detection in detections if detection["class"] in classes
            ]
            if not candidates:
                return None
            assert self.target_centroid is not None
            target = min(
                candidates,
                key=lambda detection: math.dist(
                    detection["centroid_px"], self.target_centroid
                ),
            )
            gate = self.association_gate_fraction * max(width, height)
            if math.dist(target["centroid_px"], self.target_centroid) > gate:
                return None
        if target["id"] != self.target_track:
            self.lead.reset()
        self.target_track = target["id"]
        self.target_centroid = tuple(target["centroid_px"])
        return target

    def _publish_outputs(
        self,
        source: Image,
        frame: np.ndarray | None,
        target: dict[str, Any] | None,
        detections: list[dict[str, Any]],
        latency_ms: float,
        width: int,
        height: int,
        capture_time: float,
    ) -> None:
        # Per detection: the image it came from, so a UI can draw and pick
        # boxes on the matching camera frame.
        stamp = stamp_seconds(source.header.stamp)
        public_detections = [
            {
                **{key: value for key, value in detection.items() if key != "mask"},
                "selected": detection is target,
                "stamp": stamp,
                "image_width": width,
                "image_height": height,
            }
            for detection in detections
        ]
        self.segments_pub.publish(String(data=json.dumps(public_detections)))
        fx, fy, cx, cy = self._intrinsics(width, height)
        pixel_x = pixel_y = angular_x = angular_y = confidence = 0.0
        locked = False
        selected_id = ""
        aim_error_deg = None
        bbox: list[float] = []
        mask = np.zeros((height, width), dtype=np.uint8)
        too_close = False
        if target is not None:
            aim_x, aim_y = target["centroid_px"]
            if self.actuator_fitted:
                (aim_x, aim_y), too_close = self._aim_point(target, fy, height)
            pixel_x = float(aim_x) - cx
            pixel_y = float(aim_y) - cy
            angular_x = math.atan2(pixel_x, fx)
            angular_y = -math.atan2(pixel_y, fy)
            confidence = float(target["confidence"])
            bbox = [float(value) for value in target["bbox_xywh"]]
            mask = target["mask"]
            selected_id = self.target_id if self.selection == "startup" else target["id"]
            locked = math.hypot(pixel_x, pixel_y) <= self.lock_tolerance
            aim_error_deg = math.degrees(math.atan(math.hypot(pixel_x / fx, pixel_y / fy)))
            self._publish_command(angular_x, angular_y, capture_time)
        else:
            self.lead_rate = None
        self.last_frame = {
            "time": self._now(),
            "width": width,
            "height": height,
            "center": (cx, cy),
            "detections": len(detections),
            "target": target is not None,
            "target_id": target["id"] if target is not None else "",
            "locked": locked,
            "aim_error_deg": aim_error_deg,
            "too_close": too_close,
        }
        if locked and self.fire_cycle.running and target["id"] == self.fire_cycle.target_id:
            self.tag_seen_time = self.last_frame["time"]

        self.mask_pub.publish(mask_message(mask, source))
        self.bbox_pub.publish(Float32MultiArray(data=bbox))
        self.confidence_pub.publish(Float32(data=confidence))
        self.pixel_error_pub.publish(Float32MultiArray(data=[pixel_x, pixel_y]))
        self.angular_error_pub.publish(
            Float32MultiArray(data=[angular_x, angular_y])
        )
        self.selected_id_pub.publish(String(data=selected_id))
        self.locked_pub.publish(Bool(data=locked))
        if frame is not None and (self.publish_annotated_image or self.show_window):
            annotated = self._annotate(
                frame, detections, target, latency_ms, locked, cx, cy
            )
            if self.publish_annotated_image:
                self.annotated_pub.publish(bgr_message(annotated, source))
            if self.show_window:
                self._show(annotated)
        self._publish_diagnostics(latency_ms, len(detections), target is not None)

    def _annotate(
        self,
        frame: np.ndarray,
        detections: list[dict[str, Any]],
        target: dict[str, Any] | None,
        latency_ms: float,
        locked: bool,
        center_x: float,
        center_y: float,
    ) -> np.ndarray:
        annotated = frame.copy()
        for detection in detections:
            is_target = detection is target
            color = (40, 220, 40) if is_target else (0, 180, 255)
            mask = detection["mask"] > 0
            if np.any(mask):
                annotated[mask] = (
                    0.55 * annotated[mask]
                    + 0.45 * np.asarray(color, dtype=np.float32)
                ).astype(np.uint8)

            box_x, box_y, box_w, box_h = detection["bbox_xywh"]
            x1 = max(0, int(round(box_x - box_w * 0.5)))
            y1 = max(0, int(round(box_y - box_h * 0.5)))
            x2 = min(
                annotated.shape[1] - 1, int(round(box_x + box_w * 0.5))
            )
            y2 = min(
                annotated.shape[0] - 1, int(round(box_y + box_h * 0.5))
            )
            cv2.rectangle(
                annotated, (x1, y1), (x2, y2), color, 3 if is_target else 2
            )
            label_prefix = "TARGET " if is_target else ""
            label = (
                f"{label_prefix}#{detection['id']} {detection['class']} "
                f"{detection['confidence']:.2f}"
            )
            cv2.putText(
                annotated,
                label,
                (x1, max(18, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )
            centroid = tuple(
                int(round(value)) for value in detection["centroid_px"]
            )
            cv2.circle(annotated, centroid, 4, color, -1)

        center = (int(round(center_x)), int(round(center_y)))
        cv2.drawMarker(
            annotated,
            center,
            (255, 255, 255),
            cv2.MARKER_CROSS,
            28,
            2,
        )
        status = "LOCKED" if locked else "SEARCHING"
        status_color = (40, 220, 40) if locked else (0, 180, 255)
        cv2.putText(
            annotated,
            f"{status}  {latency_ms:.1f} ms",
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _show(self, frame: np.ndarray) -> None:
        try:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.window_initialized = True
            cv2.imshow(self.window_name, frame)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                cv2.destroyWindow(self.window_name)
                self.window_initialized = False
                self.show_window = False
                self.get_logger().info(
                    "Display window closed; ROS publishers remain active"
                )
        except cv2.error as exc:
            self.get_logger().error(f"Unable to show OpenCV window: {exc}")
            self.show_window = False

    def _command_gate(self, now: float) -> str:
        """Why commands are (not) being sent: 'measured'/'commanded' allow them."""
        if not self.command_enabled:
            return "disabled"
        if self.estop_engaged:
            return "emergency_stop"
        if now <= 0.0:
            return "waiting_for_clock"  # use_sim_time and no /clock yet
        if self.controller.has_feedback(now):
            return "measured"
        if (
            self.actuator_ready
            and now - self.actuator_ready_time <= self.actuator_ready_timeout_s
        ):
            return "commanded"
        # Without measured joint states or a live actuator, integrating
        # commands that are never executed would wind the turret to a limit.
        return "actuator_not_ready"

    def _lead(
        self,
        capture_time: float,
        angular_x: float,
        angular_y: float,
        now: float,
        attitude: Attitude | None,
    ) -> np.ndarray | None:
        """Add the target's line of sight and return its rate, if usable."""
        _, source = self.controller.reference(capture_time, now)
        if (
            not self.target_lead
            or source != "measured"
            or not finite(capture_time, angular_x, angular_y)
        ):
            # Without measured joints the line of sight would include the
            # turret's own estimated motion, and its rate would feed back.
            self.lead.reset()
            return None
        if (attitude is not None) != self.lead_world:
            self.lead.reset()  # its samples were in the other frame
            self.lead_world = attitude is not None
        self.lead.add(
            capture_time,
            self.controller.sight(capture_time, angular_x, angular_y, now, attitude),
        )
        return self.lead.rate(now)

    def _publish_command(
        self, angular_x: float, angular_y: float, capture_time: float
    ) -> None:
        now = self._now()
        attitude = self._attitude()
        self.lead_rate = self._lead(capture_time, angular_x, angular_y, now, attitude)
        self.command_state = self._command_gate(now)
        if self.command_state not in COMMANDING:
            return
        command_position = self.controller.update(
            capture_time, angular_x, angular_y, now, attitude, self.lead_rate
        )
        if command_position is None:
            self.command_state = "invalid_error"
            return
        self._send(now, command_position, record=False)  # update() recorded it

    def _send(
        self, now: float, position: tuple[float, float], record: bool = True
    ) -> None:
        pan, tilt = position
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["pan_joint", "tilt_joint"]
        command.position = [pan, tilt]
        if self.actuator_fitted:
            # Every command carries the launcher: the rack back only while
            # priming and the trigger off 0 only while firing, in a cycle a
            # tag command started; otherwise the trigger at 0 and the rack
            # forward, or stopped where it is: a command without
            # actuator_joint, which the bridge and firmware hold at the
            # rack's present position (never a stale measurement of it).
            rack, trigger = self.fire_cycle.goals(now)
            if trigger > 0.0 and not self._line_of_fire_clear(now):
                trigger = 0.0  # blocked between control ticks: the next update faults the cycle
            if rack is not None:
                command.name.append(ACTUATOR_JOINT)
                command.position.append(rack)
            command.name.append(TRIGGER_JOINT)
            command.position.append(trigger)
        self.command_pub.publish(command)
        self.last_send_time = now
        if record:
            self.controller.record(now, pan, tilt)

    def _control_tick(self) -> None:
        """Between camera frames: manual rates, centring, keeping the
        moving-base aim on the target, and the dart launcher's fire cycle."""
        now = self._now()
        if self.actuator_fitted:
            self._update_tag(now)
        self._control_step(now)
        if self.actuator_fitted:
            self._launcher_keepalive(now)

    def _control_step(self, now: float) -> None:
        if self.mode == "idle" and self.goto is None:
            return
        if self.mode == "auto" and self.controller.aim is None:
            return
        gate = self._command_gate(now)
        if gate not in COMMANDING:
            self.command_state = gate
            self.manual_last = None
            return
        if self.mode == "manual":
            position = self._manual_step(now)
        elif self.mode == "auto":
            position = self._hold_step(now)
        else:
            position = self._center_step(now)
        last = self.controller.last_command
        if position is None or (
            last is not None
            and max(abs(position[0] - last[1]), abs(position[1] - last[2])) < 1e-4
        ):
            return
        self.command_state = gate
        self._send(now, position)

    def _manual_step(self, now: float) -> tuple[float, float] | None:
        if not 0.0 <= now - self.manual_time <= self.manual_timeout_s:
            self.manual_last = None  # deadman: hold where the turret is
            return None
        if self.manual_last is None:
            start, dt = self.controller.position(now)[1:], self.control_period_s
        else:
            start, last = self.manual_last
            dt = min(max(now - last, 0.0), 0.1)
        position = tuple(
            clamp(value + rate * dt, low, high)
            for value, rate, (low, high) in zip(
                start, self.manual_rates, self.controller.limits
            )
        )
        self.manual_last = (position, now)
        return position

    def _center_step(self, now: float) -> tuple[float, float] | None:
        assert self.goto is not None
        start = self.controller.position(now)[1:]
        step = self.controller.max_rate * self.control_period_s
        position = tuple(
            value + max(-step, min(step, goal - value))
            for value, goal in zip(start, self.goto)
        )
        if position == self.goto:
            self.goto = None
            self._notify(now, "centred")
        return position

    def _hold_step(self, now: float) -> tuple[float, float] | None:
        if not self.controller.continuing(now):
            return None  # after a pause the next frame re-aims gradually
        return self.controller.hold(
            now + self.controller.actuation_delay(now), self._attitude()
        )

    def _turret_command(self, msg: String) -> None:
        now = self._now()
        try:
            command = json.loads(msg.data)
        except (TypeError, ValueError):
            command = None
        action = command.get("action") if isinstance(command, dict) else None
        if action in ("lock", "release", "manual", "center", "next"):
            # Anything that moves the aim ends a fire cycle in progress.
            self._abort_tag(now, f"{action} command")
        if action == "tag":
            self._tag(command, now)
        elif action == "next":
            self._next(now)
        elif action == "clear_tags":
            self.tagged_ids.clear()
            self._notify(now, "tags cleared")
        elif action == "auto_next":
            self._set_auto_next(command, now)
        elif action == "reload":
            self._reload(command, now)
        elif action == "lock":
            self._lock(command, now)
        elif action == "release":
            self._set_mode("idle")
            self._notify(now, "released")
        elif action == "manual":
            self._manual(command, now)
        elif action == "center":
            self._set_mode("idle")
            self.goto = (
                clamp(0.0, *self.controller.limits[0]),
                clamp(0.0, *self.controller.limits[1]),
            )
            self._notify(now, "centring")
        else:
            self._notify(now, f"ignored command {str(msg.data)[:80]!r}")
        self._publish_status()

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.goto = None
        self.manual_last = None
        self.lead.reset()
        if mode != "auto":
            self.selection = ""
            self.target_track = ""
            self.controller.aim = None

    def _lock(self, command: dict[str, Any], now: float) -> None:
        if "target_id" in command:
            track_id = str(command["target_id"])
            track = self.tracks.get(track_id)
            if track is None:
                self._notify(now, f"no detection with id {track_id}")
                return
            self._select(now, track_id, track["class"], track["centroid_px"])
            return
        point = command.get("point")
        if (
            not isinstance(point, (list, tuple))
            or len(point) != 2
            or not finite(*point)
            or not all(0.0 <= float(value) <= 1.0 for value in point)
        ):
            self._notify(now, "lock needs target_id or point [u, v] in [0, 1]")
            return
        if self.last_frame is None:
            self._notify(now, "no camera image yet")
            return
        u, v = float(point[0]), float(point[1])
        x, y = u * self.last_frame["width"], v * self.last_frame["height"]
        containing = [
            detection
            for detection in self.last_detections
            if abs(x - detection["bbox_xywh"][0]) <= detection["bbox_xywh"][2] * 0.5
            and abs(y - detection["bbox_xywh"][1]) <= detection["bbox_xywh"][3] * 0.5
        ]
        if not containing:
            self._notify(now, f"no detection at ({u:.2f}, {v:.2f})")
            return
        detection = min(
            containing, key=lambda d: math.dist(d["bbox_xywh"][:2], (x, y))
        )
        self._select(now, detection["id"], detection["class"], detection["centroid_px"])

    def _select(
        self, now: float, track_id: str, target_class: str, centroid: list[float]
    ) -> None:
        self._set_mode("auto")
        self.selection = "track"
        self.target_track = track_id
        self.lock_class = target_class
        self.target_centroid = (float(centroid[0]), float(centroid[1]))
        self._notify(now, f"locked on {target_class} {track_id}")

    def _manual(self, command: dict[str, Any], now: float) -> None:
        try:
            rates = (float(command.get("pan_rate", 0.0)), float(command.get("tilt_rate", 0.0)))
        except (TypeError, ValueError):
            rates = (math.nan, math.nan)
        if not finite(*rates):
            self._notify(now, "manual needs numeric pan_rate and tilt_rate")
            return
        if self.mode != "manual":
            self._set_mode("manual")
            self._notify(now, "manual control")
        limit = self.controller.max_rate
        self.manual_rates = tuple(clamp(rate, -limit, limit) for rate in rates)
        self.manual_time = now

    def _notify(self, now: float, text: str) -> None:
        self.notice = (text, now)
        self.get_logger().info(text)

    # --- Foam dart launcher ------------------------------------------------

    def _aim_point(
        self, target: dict[str, Any], fy: float, height: int
    ) -> tuple[tuple[float, float], bool]:
        """Where to aim at a target, in pixels, and whether it is too close.
        A person: the torso, person_aim_fraction of the box height from the
        top (never the head), raised by the drop offset for its range; else
        the mask's centroid."""
        if target["class"] != "person":
            return (float(target["centroid_px"][0]), float(target["centroid_px"][1])), False
        box_x, box_y, _, box_h = (float(value) for value in target["bbox_xywh"])
        aim_y = box_y - box_h / 2.0 + self.person_aim_fraction * box_h
        offset = self._aim_offset_rad(self.person_height_m * fy / max(box_h, 1.0))
        aim_y -= fy * math.tan(offset)
        return (box_x, aim_y), box_h > self.person_max_box_fraction * height

    def _aim_offset_rad(self, range_m: float) -> float:
        """The pitch offset (up) for the dart's drop at range_m, from the
        table, held at its ends."""
        ranges, pitches = self.aim_offset_table
        return math.radians(float(np.interp(range_m, ranges, pitches)))

    def _line_of_fire_clear(self, now: float) -> bool:
        """Whether a dart fired now misses the vehicle: the measured pan and
        tilt (fresh), and the last command while it is current, outside
        every no-fire zone widened by the margin. Without zones, always."""
        if not self.no_fire_zones:
            return True
        if not self.controller.has_feedback(now):
            return False  # where the turret points is not known
        positions = [self.controller.measured[-1][1:]]
        if self.controller.continuing(now):
            positions.append(self.controller.last_command[1:])
        return not any(in_no_fire_zone(self.no_fire_zones, pan, tilt, self.no_fire_margin)
                       for pan, tilt in positions)

    def _launcher_position(self, now: float) -> tuple[float, float] | None:
        """The measured (rack, trigger), if fresh."""
        if self.launcher_measured is None:
            return None
        stamp, rack, trigger = self.launcher_measured
        return (rack, trigger) if 0.0 <= now - stamp <= self.joint_state_timeout_s else None

    def _actuator_ready_fresh(self, now: float) -> bool:
        return (
            self.actuator_ready
            and now - self.actuator_ready_time <= self.actuator_ready_timeout_s
        )

    def _launcher_state(self, now: float) -> str:
        if not self.actuator_fitted:
            return "none"
        phase = self.fire_cycle.phase
        if phase in (FireCycle.FAULT, FireCycle.RETURNING) or self._launcher_position(now) is None:
            return "fault"
        if phase == FireCycle.RESETTING:
            return "firing"
        if phase == FireCycle.READY:
            return "ready" if self.darts > 0 else "empty"
        return phase

    def _fire_refusal(self, now: float, target_id: Any) -> str:
        """Why firing at target_id would be refused now; "" when every gate holds."""
        if not self.actuator_fitted:
            return "no launcher"
        if self.estop_engaged:
            return "emergency stop engaged"
        if not self._actuator_ready_fresh(now):
            return "actuator not ready"
        gate = self._command_gate(now)
        if gate not in COMMANDING:
            return gate.replace("_", " ")
        if self.mode != "auto" or not self.target_track:
            return "no locked target"
        if isinstance(target_id, bool) or not isinstance(target_id, (str, int)) or (
            str(target_id) == ""
        ):
            return "tag needs the locked target's target_id"
        target_id = str(target_id)
        if target_id != self.target_track:
            return f"{target_id} is not the locked target ({self.target_track})"
        frame = self.last_frame
        if (
            frame is None
            or not 0.0 <= now - frame["time"] <= self.tag_lock_timeout_s
            or frame["target_id"] != target_id
            or not frame["locked"]
        ):
            return f"{target_id} is not locked"
        if frame["too_close"]:
            return "too close"
        error = frame["aim_error_deg"]
        if error is None or not error < self.tag_max_aim_error_deg:
            return f"aim error over {self.tag_max_aim_error_deg:g} deg"
        cycle = self.fire_cycle
        if cycle.phase == FireCycle.FAULT:
            return f"launcher fault: {cycle.reason}"
        if cycle.running:
            return "a fire cycle is running"
        measured = self._launcher_position(now)
        if measured is None:
            return "no launcher position"
        if cycle.phase != FireCycle.READY or not cycle.at_rest(*measured):
            return "launcher not at rest"
        if self.darts <= 0:
            return "no darts"
        if not self._line_of_fire_clear(now):
            return "line of fire blocked"
        wait = self.tag_min_interval_s - (now - self.last_cycle_end)
        if wait > 0.0:
            return f"{wait:.1f} s to the next shot"
        return ""

    def _tag(self, command: dict[str, Any], now: float) -> None:
        """{"action": "tag", "target_id": id}: one shot, if every gate holds."""
        target_id = command.get("target_id")
        refusal = self._fire_refusal(now, target_id)
        if refusal:
            self.last_tag = {
                "target_id": "" if target_id is None else str(target_id),
                "result": "refused",
                "reason": refusal,
                "stamp": now,
            }
            self._notify(now, f"fire refused: {refusal}")
            return
        target_id = self.target_track
        self.fire_cycle.start(now, target_id)
        self.auto_next_pending = True
        self.tag_seen_time = now
        self.last_tag = {"target_id": target_id, "result": "running", "reason": "", "stamp": now}
        self._notify(now, f"firing at {target_id}")
        self._launcher_keepalive(now, force=True)

    def _abort_tag(self, now: float, reason: str) -> None:
        # Even after the dart released, an operator's stop/selection wins
        # over advancing when the trigger finishes returning to rest.
        self.auto_next_pending = False
        cycle = self.fire_cycle
        target_id = cycle.target_id
        if not cycle.running:
            return
        event = cycle.abort(now, reason)
        if event is not None:
            self._tag_ended(now, *event)
        else:
            self._notify(now, f"{reason} after the shot at {target_id}: trigger back to 0")
        self._launcher_keepalive(now, force=True)

    def _update_tag(self, now: float) -> None:
        cycle = self.fire_cycle
        if cycle.running and cycle.phase in (
            FireCycle.PRIMING, FireCycle.CHAMBERING, FireCycle.FIRING
        ):
            if not self._actuator_ready_fresh(now):
                self._abort_tag(now, "actuator not ready")
            elif self._command_gate(now) not in COMMANDING:
                self._abort_tag(now, self._command_gate(now).replace("_", " "))
            elif (
                self.mode != "auto"
                or self.target_track != cycle.target_id
                or now - self.tag_seen_time > self.tag_lock_timeout_s
            ):
                self._abort_tag(now, f"lost the lock on {cycle.target_id}")
        measured = self._launcher_position(now)
        event = cycle.update(now, *(measured if measured is not None else (None, None)),
                             clear=self._line_of_fire_clear(now))
        if event is not None:
            self._tag_ended(now, *event)

    def _tag_ended(
        self, now: float, target_id: str, result: str, reason: str, shot: bool
    ) -> None:
        advance = self.auto_next_pending
        self.auto_next_pending = False
        if shot:
            self.darts = max(0, self.darts - 1)
            self.shots += 1
        if target_id:
            self.last_cycle_end = now
            self.last_tag = {
                "target_id": target_id, "result": result, "reason": reason, "stamp": now
            }
        if result == "fault":
            self._notify(now, f"launcher fault: {reason}")
            # Stop the rack and return the trigger at once, not when the
            # bridge's watchdog notices the commands have stopped.
            self._launcher_keepalive(now, force=True)
        elif result == "fired":
            if target_id not in self.tagged_ids:
                self.tagged_ids.append(target_id)
            self._notify(now, f"fired at {target_id}; {self.darts} darts left")
            if (self.auto_next and advance and self.mode == "auto"
                    and self.target_track == target_id and self._command_gate(now) in COMMANDING):
                self._next(now)

    def _launcher_keepalive(self, now: float, force: bool = False) -> None:
        """Keep commands carrying the launcher's goals flowing while it is
        not verified at rest, holding pan and tilt where they are going;
        force: send one now (a cycle started or stopped)."""
        cycle = self.fire_cycle
        measured = self._launcher_position(now)
        if not force and (
            cycle.phase == FireCycle.READY
            or (cycle.phase == FireCycle.FAULT
                and (measured is None or measured[1] <= cycle.trigger_tolerance))
            or now - self.last_send_time < LAUNCHER_KEEPALIVE_S
        ):
            return
        if self._command_gate(now) not in COMMANDING:
            return  # the bridge's STOP or watchdog makes it safe meanwhile
        if self.controller.continuing(now):
            position = self.controller.last_command[1:]
        elif self.controller.has_feedback(now):
            position = self.controller.measured[-1][1:]
        else:
            return
        self._send(now, position, record=False)

    def _reload(self, command: dict[str, Any], now: float) -> None:
        darts = command.get("darts")
        if isinstance(darts, bool) or not isinstance(darts, int) or not 0 <= darts <= 1000:
            self._notify(now, "reload needs darts: a whole number from 0")
            return
        if self.fire_cycle.running:
            self._notify(now, "reload refused: a fire cycle is running")
            return
        self.darts = darts
        self._notify(now, f"reloaded: {darts} darts")

    def _next(self, now: float) -> None:
        """Lock (aim at) the untagged detection nearest the image centre.
        It never tags: that needs another tag command."""
        frame = self.last_frame
        if frame is None or not 0.0 <= now - frame["time"] <= 1.0:
            self._notify(now, "next: no camera images")
            return
        cx, cy = frame["center"]
        candidates = [
            detection
            for detection in self.last_detections
            if detection["id"] not in self.tagged_ids and detection["id"] != self.target_track
        ]
        if not candidates:
            self._notify(now, "next: no untagged detection")
            return
        detection = min(
            candidates, key=lambda d: math.dist(d["centroid_px"], (cx, cy))
        )
        self._select(now, detection["id"], detection["class"], detection["centroid_px"])

    def _set_auto_next(self, command: dict[str, Any], now: float) -> None:
        enabled = command.get("enabled")
        if not isinstance(enabled, bool):
            self._notify(now, "auto_next needs enabled: true or false")
            return
        self.auto_next = enabled
        self._notify(now, f"auto next {'on' if enabled else 'off'}")

    def _status(self, now: float) -> dict[str, Any]:
        frame = self.last_frame
        recent = frame is not None and 0.0 <= now - frame["time"] <= 1.0
        auto = self.mode == "auto"
        gate = self._command_gate(now)
        messages = []
        text, noticed = self.notice
        if text and 0.0 <= now - noticed <= 3.0:
            messages.append(text)
        if gate == "disabled":
            messages.append("commands disabled")
        elif gate == "emergency_stop":
            messages.append("emergency stop engaged")
        elif gate == "waiting_for_clock":
            messages.append("waiting for /clock")
        elif gate == "actuator_not_ready" and (self.mode != "idle" or self.goto):
            messages.append("actuator not ready: no joint states or actuator_ready")
        if not recent:
            messages.append("no camera images")
        elif auto and not frame["target"]:
            messages.append(f"searching for {self._selected_class() or 'the target'}")
        if self.base_compensation and not self._compensating(now):
            problem = self.imu_problem or (
                f"no IMU on {self.base_imu_topic} for {self.imu_timeout_s:g} s"
            )
            messages.append(f"{problem}: fixed-base aiming")
        launcher = self._launcher_state(now)
        if launcher == "fault":
            reason = self.fire_cycle.reason or (
                "no launcher position in the joint states"
                if self._launcher_position(now) is None else "returning to rest"
            )
            messages.append(f"launcher fault: {reason}")
        fire_block = self._fire_refusal(now, self.target_track)
        return {
            "mode": "estop" if self.estop_engaged else self.mode,
            "target_id": self.target_track if auto else "",
            "target_class": self._selected_class() if auto else "",
            "locked": bool(auto and recent and frame["locked"]),
            "aim_error_deg": frame["aim_error_deg"] if auto and recent else None,
            "base_compensation": self._compensating(now),
            "target_lead": bool(self.target_lead and self.lead_rate is not None),
            "detections": frame["detections"] if recent else 0,
            "message": "; ".join(messages),
            "stamp": now,
            "command_state": self.command_state,
            # Foam dart launcher
            "launcher": launcher,
            "darts": self.darts,
            "tags": len(self.tagged_ids),
            "tagged_ids": list(self.tagged_ids),
            "last_tag": None if self.last_tag is None else dict(self.last_tag),
            "fire_block": fire_block,
            "auto_next": self.auto_next,
        }

    def _selected_class(self) -> str:
        if self.selection == "track":
            return self.lock_class
        return self.target_class if self.selection == "startup" else ""

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self._status(self._now()))))

    def _publish_diagnostics(
        self, latency_ms: float, detection_count: int, target_detected: bool
    ) -> None:
        status = DiagnosticStatus()
        status.name = "pantilt_yolo_tracker"
        status.hardware_id = Path(self.model_path).name
        status.level = DiagnosticStatus.OK
        status.message = "target detected" if target_detected else "target not detected"
        status.values = [
            KeyValue(key="backend", value="ultralytics"),
            KeyValue(key="model", value=self.model_path),
            KeyValue(key="device", value=self.device),
            KeyValue(key="latency_ms", value=f"{latency_ms:.3f}"),
            KeyValue(key="detection_count", value=str(detection_count)),
            KeyValue(key="frame_count", value=str(self.frame_count)),
            KeyValue(key="command_state", value=self.command_state),
            KeyValue(
                key="actuation_delay_s",
                value=f"{self.controller.actuation_delay(self._now()):.3f}",
            ),
            KeyValue(key="emergency_stop", value=str(self.estop_engaged).lower()),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def _safe_for_shutdown(self) -> None:
        """Shutting down: the trigger to 0 and the rack stopped now (the
        bridge's watchdog would 0.5 s later)."""
        try:
            if self.actuator_fitted and self.fire_cycle.phase != FireCycle.READY:
                now = self._now()
                self._abort_tag(now, "shutdown")
                self._launcher_keepalive(now, force=True)
        except Exception:  # the ROS context may already be shutting down
            pass

    def destroy_node(self) -> bool:
        self._safe_for_shutdown()
        try:
            if self.window_initialized:
                cv2.destroyWindow(self.window_name)
            self.ready_file.unlink(missing_ok=True)
        finally:
            return super().destroy_node()


def main() -> None:
    os.environ.setdefault("YOLO_CONFIG_DIR", "/workspace/cache/ultralytics")
    rclpy.init()
    node = PanTiltYoloTracker()
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

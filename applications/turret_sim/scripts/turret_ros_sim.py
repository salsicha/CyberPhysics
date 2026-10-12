#!/usr/bin/env python3
"""ROS 2 camera and actuator simulator for the shared PanTiltROS YOLO node.

With --actuator true the turret also carries the foam dart launcher
(systems/racecar_turret/interfaces.md, "Foam dart launcher"): actuator_joint
(the priming rack) and trigger_joint in the joint states and commands, moving
at the ST3215's speed, with the sear between them (a shot when the trigger is
driven to the fire angle after the rack was driven to the full draw), the
trigger to 0 and the rack stopped by the emergency stop and when commands
stop for 0.5 s (as the firmware does), and /turret/actuator_ready. --tag-at T
plays the operator: from T seconds on it reloads ({"action": "reload",
"darts": N}) and, when /turret/status says the Fire button is enabled
(fire_block ""), presses it ({"action": "tag", "target_id": <locked id>}), up
to --tag-attempts times, until the clip's N darts are shot; Fire never moves
the aim. With --tag-lock true it first taps the tracked target's box
({"action": "lock", "target_id": <id>}), so the tracker follows that one
detection, which changes the tracking, and acknowledges a launcher fault by
disarming and arming (0.5 s of /turret/emergency_stop) so it can fire again.
The report's "launcher" section records the presses, the shots, each shot's
cycle time and the true aim error at it; the tracking summary and its
scoring are unchanged.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from turret_sim import TurretSim, percentile


def image_message(frame: np.ndarray, stamp: Any, frame_id: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(msg.width) * 3
    msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
    return msg


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
ACTUATOR_JOINT = "actuator_joint"
TRIGGER_JOINT = "trigger_joint"
# How long the operator disarms the turret to acknowledge a launcher fault
OPERATOR_DISARM_S = 0.5
# The tracker's contract topics outside the scenario's topic list
OPERATOR_TOPICS = {
    "actuator_ready": "/turret/actuator_ready",
    "command": "/turret/command",
    "status": "/turret/status",
}


def str2bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def mono_image(msg: Image) -> np.ndarray | None:
    if msg.encoding != "mono8" or msg.step < msg.width:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        return None
    return raw[:expected].reshape(int(msg.height), int(msg.step))[:, : int(msg.width)].copy()


class TurretRosSimulator(Node):
    # Launcher defaults (the metric tests build the node without __init__)
    actuator_fitted = False
    tag_at: float | None = None

    def __init__(
        self,
        scenario: dict[str, Any],
        output: Path,
        variant: str,
        save_frames: Path | None,
        actuator: bool = False,
        tag_at: float | None = None,
        tag_attempts: int = 10,
        tag_lock: bool = False,
        darts: int = 3,
    ) -> None:
        super().__init__("turret_ros_sim")
        self.scenario = scenario
        self.output = output
        self.variant = variant
        self.save_frames = save_frames
        if save_frames is not None:
            save_frames.mkdir(parents=True, exist_ok=True)

        self.actuator_fitted = actuator
        self.tag_at = tag_at
        self.tag_attempts = int(tag_attempts)
        self.tag_lock = bool(tag_lock)
        self.darts = int(darts)
        self.sim = TurretSim(scenario, variant_name=variant, launcher=actuator)
        self.sim.cv2 = cv2
        self.sim.np = np
        self.sim.background = self.sim._render_background()
        self.dt = float(scenario["dt_s"])
        self.steps = int(float(scenario["duration_s"]) / self.dt) + 1
        self.frame_index = 0
        self.current_t = 0.0
        self.finished = False
        self.operator_disarms = []
        self.input_frames = {}
        self.frame_results = {}
        self.drain_started = None
        self.discovery_started = time.monotonic()
        self.discovery_ready = False
        self.discovery_wait_s = 0.0
        self.records: list[dict[str, Any]] = []
        self.reacquire_times: list[float] = []
        self.occlusion_started: float | None = None
        self.was_detected = False
        self.false_lock_count = 0
        # (time, saturated) for every accepted joint command.
        self.command_log: list[tuple[float, bool]] = []
        self.dropped_command_count = 0
        self.emergency_stop_started: float | None = None
        # Commands already in flight when the stop is raised are not violations.
        self.emergency_stop_grace_s = float(
            scenario["pantilt"].get("emergency_stop_grace_s", 2.0 * self.dt)
        )

        self.latest: dict[str, Any] = {
            "segments": [],
            "mask": None,
            "bbox": [],
            "confidence": 0.0,
            "pixel_error": [0.0, 0.0],
            "angular_error": [0.0, 0.0],
            "selected_id": "",
            "locked": False,
            "latency_ms": 0.0,
        }
        topics = scenario["topics"]
        self.image_pub = self.create_publisher(
            Image, topics["camera_image"], qos_profile_sensor_data
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, topics["camera_info"], qos_profile_sensor_data
        )
        self.joint_state_pub = self.create_publisher(
            JointState, topics["joint_states"], qos_profile_sensor_data
        )
        self.estop_pub = self.create_publisher(Bool, topics["emergency_stop"], LATCHED_QOS)
        self.create_subscription(
            JointState, topics["joint_commands"], self._joint_command, 10
        )
        self.create_subscription(String, topics["segments"], self._segments, 10)
        self.create_subscription(Image, topics["target_mask"], self._mask, 10)
        self.create_subscription(
            Float32MultiArray, topics["target_bbox"], self._bbox, 10
        )
        self.create_subscription(
            Float32, topics["target_confidence"], self._confidence, 10
        )
        self.create_subscription(
            Float32MultiArray, topics["pixel_error"], self._pixel_error, 10
        )
        self.create_subscription(
            Float32MultiArray, topics["angular_error"], self._angular_error, 10
        )
        self.create_subscription(String, topics["selected_id"], self._selected_id, 10)
        self.create_subscription(Bool, topics["locked"], self._locked, 10)
        self.create_subscription(
            DiagnosticArray, topics["diagnostics"], self._diagnostics, 10
        )
        if self.actuator_fitted:
            self._init_tagging(topics)
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"Turret ROS simulation ready: {self.steps} frames at {1.0 / self.dt:.1f} Hz"
        )

    def _init_tagging(self, topics: dict[str, str]) -> None:
        names = {key: topics.get(key, default) for key, default in OPERATOR_TOPICS.items()}
        self.ready_pub = self.create_publisher(Bool, names["actuator_ready"], LATCHED_QOS)
        self.operator_pub = self.create_publisher(String, names["command"], 10)
        for qos in (LATCHED_QOS, 10):
            self.create_subscription(String, names["status"], self._status, qos)
        self.status: dict[str, Any] | None = None
        self.status_stamp = -math.inf  # the newest status accepted
        self.presses: list[dict[str, Any]] = []
        self.locks: list[dict[str, Any]] = []  # the operator's lock taps
        self.last_tags: list[dict[str, Any]] = []  # each change of the status's last_tag
        self.reloaded = False
        # With --tag-lock: the operator's disarm intervals (start, end) that
        # acknowledge launcher faults, and the faults acknowledged
        self.operator_disarms: list[tuple[float, float]] = []
        self.acknowledged: list[dict[str, Any]] = []
        # (t, rack, trigger, the status's launcher state)
        self.launcher_log: list[tuple[float, float, float, str]] = []
        self.launcher_truth: dict[float, dict[str, Any]] = {}
        # Times of commands that prime (the rack further back than it is) or fire
        self.priming_commands: list[float] = []
        self.firing_commands: list[float] = []

    def _status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(status, dict):
            return
        stamp = status.get("stamp")
        if isinstance(stamp, (int, float)) and not isinstance(stamp, bool):
            # Both subscriptions get every message, and a copy can arrive
            # after a newer status: keep only newer ones
            if stamp <= self.status_stamp:
                return
            self.status_stamp = float(stamp)
        self.status = status
        last = status.get("last_tag")
        if isinstance(last, dict) and (not self.last_tags or self.last_tags[-1]["tag"] != last):
            self.last_tags.append({"t_s": self.current_t, "tag": last})

    def _operate(self) -> None:
        """The operator: reload, tap the target's box (with --tag-lock), then
        press Fire when the page would enable it, until the clip is shot.
        Never a tap while a cycle runs (a lock stops it). With --tag-lock a
        launcher fault is acknowledged as on the page: disarm, then arm."""
        status = self.status
        if self.tag_at is None or self.current_t < self.tag_at or status is None:
            return
        if not self.reloaded:
            self.operator_pub.publish(
                String(data=json.dumps({"action": "reload", "darts": self.darts}))
            )
            self.reloaded = True
            return
        results = [entry["tag"].get("result") for entry in self.last_tags]
        if results.count("fired") >= self.darts or len(self.presses) >= self.tag_attempts:
            return
        faults = [entry["t_s"] for entry in self.last_tags if entry["tag"].get("result") == "fault"]
        if self.tag_lock and len(faults) > len(self.acknowledged):
            # A fault latches until the turret is disarmed and armed again.
            # The operator does that half a second later, clearing a primed
            # launcher by hand first (no shot), as the docs require.
            if self.current_t - faults[-1] >= 0.5:
                self.operator_disarms.append((self.current_t, self.current_t + OPERATOR_DISARM_S))
                self.acknowledged.append({"t_s": self.current_t, "cleared_primed": self.sim.primed})
                self.sim.primed = False
                self.get_logger().info(f"operator: disarm and arm at {self.current_t:.2f} s")
            return
        if results and results[-1] == "running" or status.get("launcher") != "ready":
            return  # a cycle is under way, or the launcher is not ready
        if self.presses and self.current_t - self.presses[-1]["t_s"] < 1.0:
            return
        target_id = status.get("target_id")
        if not (status.get("locked") and target_id):
            return
        if target_id in status.get("tagged_ids", []):
            return
        last_lock = self.locks[-1] if self.locks else None
        if not self.tag_lock:
            pass  # press Tag on whatever the tracker follows
        elif last_lock is None or last_lock["target_id"] != target_id:
            # Follow this detection by its ID, as a tap on its box does
            if last_lock is None or self.current_t - last_lock["t_s"] >= 1.0:
                self.operator_pub.publish(
                    String(data=json.dumps({"action": "lock", "target_id": target_id}))
                )
                self.locks.append({"t_s": self.current_t, "target_id": target_id})
            return
        if self.tag_lock and self.current_t - last_lock["t_s"] < 0.3:
            return
        if status.get("fire_block") != "":
            return
        self.operator_pub.publish(
            String(data=json.dumps({"action": "tag", "target_id": target_id}))
        )
        self.presses.append({
            "t_s": self.current_t,
            "target_id": target_id,
            "aim_error_deg": status.get("aim_error_deg"),
        })
        self.get_logger().info(f"operator: fire at {target_id} at {self.current_t:.2f} s")

    def _joint_command(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if "pan_joint" not in positions or "tilt_joint" not in positions:
            return
        pan = float(positions["pan_joint"])
        tilt = float(positions["tilt_joint"])
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            return
        launcher = None  # (rack or None: stop it, trigger)
        if self.actuator_fitted:
            rack = positions.get(ACTUATOR_JOINT)
            trigger = positions.get(TRIGGER_JOINT, 0.0) if rack is not None else 0.0
            if not all(math.isfinite(value) for value in (rack or 0.0, trigger)):
                return
            launcher = (None if rack is None else float(rack), float(trigger))
        if self.sim.command_dropped(self.current_t):
            self.dropped_command_count += 1
            return
        if (
            self.emergency_stop_started is not None
            and self.current_t - self.emergency_stop_started > self.emergency_stop_grace_s
        ):
            # The controller must stop commanding while the e-stop is raised.
            self.sim.emergency_stop_violations += 1
            return
        pan_limits = [float(value) for value in self.scenario["pantilt"]["pan_limit_rad"]]
        tilt_limits = [float(value) for value in self.scenario["pantilt"]["tilt_limit_rad"]]
        saturated = (
            pan <= pan_limits[0] + 1e-6 or pan >= pan_limits[1] - 1e-6
            or tilt <= tilt_limits[0] + 1e-6 or tilt >= tilt_limits[1] - 1e-6
        )
        self.command_log.append((self.current_t, saturated))
        pan = max(pan_limits[0], min(pan_limits[1], pan))
        tilt = max(tilt_limits[0], min(tilt_limits[1], tilt))
        execute_at = self.current_t + float(self.scenario["pantilt"]["command_latency_s"])
        self.sim.command_queue.append((execute_at, pan, tilt))
        if launcher is not None:
            # Like the bridge: without actuator_joint the trigger goes to 0
            # and the rack stops where it is
            self.sim.launcher_queue.append((execute_at, *launcher))
            rack, trigger = launcher
            if rack is not None and rack > self.sim.rack + 1e-9:
                self.priming_commands.append(self.current_t)
            if trigger > 0.0:
                self.firing_commands.append(self.current_t)

    def _segments(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            self.latest["segments"] = value if isinstance(value, list) else []
        except json.JSONDecodeError:
            self.latest["segments"] = []

    def _mask(self, msg: Image) -> None:
        self.latest["mask"] = mono_image(msg)
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        source = self.input_frames.get(stamp)
        if source is not None and stamp not in self.frame_results:
            target, scenario_time = source
            predicted = self.latest['mask']
            visible = target is not None and bool(np.any(target))
            nonempty = predicted is not None and bool(np.any(predicted))
            self.frame_results[stamp] = {
                't_s': scenario_time, 'target_visible': visible,
                'mask_present': nonempty,
                'iou': self.sim._mask_iou(predicted, target) if nonempty and visible else 0.0,
                'capture_to_result_ms': (self.get_clock().now().nanoseconds - stamp) / 1e6,
            }

    def _bbox(self, msg: Float32MultiArray) -> None:
        self.latest["bbox"] = [float(value) for value in msg.data]

    def _confidence(self, msg: Float32) -> None:
        self.latest["confidence"] = float(msg.data)

    def _pixel_error(self, msg: Float32MultiArray) -> None:
        self.latest["pixel_error"] = [float(value) for value in msg.data[:2]]

    def _angular_error(self, msg: Float32MultiArray) -> None:
        self.latest["angular_error"] = [float(value) for value in msg.data[:2]]

    def _selected_id(self, msg: String) -> None:
        self.latest["selected_id"] = msg.data

    def _locked(self, msg: Bool) -> None:
        self.latest["locked"] = bool(msg.data)

    def _diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "pantilt_yolo_tracker":
                continue
            values = {value.key: value.value for value in status.values}
            try:
                self.latest["latency_ms"] = float(values.get("latency_ms", 0.0))
            except ValueError:
                self.latest["latency_ms"] = 0.0

    def _tick(self) -> None:
        if self.finished:
            return
        if not self.discovery_ready:
            # Model warmup does not imply that this newly-created camera and
            # result subscriber have discovered the tracker yet. Start the
            # scenario only after both directions of the graph are connected.
            self.discovery_wait_s = time.monotonic() - self.discovery_started
            topics = self.scenario["topics"]
            self.discovery_ready = (
                self.image_pub.get_subscription_count() > 0
                and self.camera_info_pub.get_subscription_count() > 0
                and self.joint_state_pub.get_subscription_count() > 0
                and self.count_publishers(topics["target_mask"]) > 0
            )
            if not self.discovery_ready:
                if self.discovery_wait_s > 30.0:
                    raise RuntimeError("Tracker camera/state/result graph did not connect within 30 s")
                return
        if self.frame_index >= self.steps:
            # Allow the final images to leave the tracker queue before counting
            # input frames with no result. No new image/plant steps are sent.
            self.drain_started = self.drain_started or time.monotonic()
            if time.monotonic() - self.drain_started >= 2.0:
                self._finish()
            return
        self.current_t = round(self.frame_index * self.dt, 6)
        emergency_stop = self.sim.in_emergency_stop(self.current_t) or any(
            start <= self.current_t < end for start, end in self.operator_disarms
        )
        if not emergency_stop:
            self.emergency_stop_started = None
        elif self.emergency_stop_started is None:
            self.emergency_stop_started = self.current_t
        self.sim.update_servo(self.current_t, self.dt, emergency_stop)
        frame, ground_truth_masks = self.sim.render_frame(self.current_t)
        if self.actuator_fitted:
            self._record_launcher_truth(ground_truth_masks)
        self._record(ground_truth_masks, emergency_stop)
        stamp = self.get_clock().now().to_msg()
        self._publish_state(stamp, emergency_stop)
        if self.actuator_fitted:
            self.launcher_log.append((
                self.current_t, self.sim.rack, self.sim.trigger,
                str((self.status or {}).get("launcher", "")),
            ))
            self._operate()
        if not self.sim.camera_frame_dropped(self.current_t):
            key = stamp.sec * 1_000_000_000 + stamp.nanosec
            self.input_frames[key] = (ground_truth_masks.get(self.sim.target_id), self.current_t)
            self.image_pub.publish(
                image_message(frame, stamp, self.scenario["camera"]["frame_id"])
            )
            self.camera_info_pub.publish(self._camera_info(stamp))
        if self.save_frames is not None and (
            self.frame_index % 20 == 0 or self.frame_index == self.steps - 1
        ):
            cv2.imwrite(str(self.save_frames / f"frame_{self.frame_index:05d}.jpg"), frame)
        self.frame_index += 1

    def _camera_info(self, stamp: Any) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.scenario["camera"]["frame_id"]
        msg.width = self.sim.width
        msg.height = self.sim.height
        msg.k = [
            self.sim.fx,
            0.0,
            self.sim.cx,
            0.0,
            self.sim.fy,
            self.sim.cy,
            0.0,
            0.0,
            1.0,
        ]
        msg.p = [
            self.sim.fx,
            0.0,
            self.sim.cx,
            0.0,
            0.0,
            self.sim.fy,
            self.sim.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        msg.distortion_model = "plumb_bob"
        return msg

    def _publish_state(self, stamp: Any, emergency_stop: bool) -> None:
        state = JointState()
        state.header.stamp = stamp
        state.name = ["pan_joint", "tilt_joint"]
        state.position = [self.sim.pan, self.sim.tilt]
        state.velocity = [0.0, 0.0]
        state.effort = [0.0, 0.0]
        if self.actuator_fitted:
            # (extend: rclpy keeps these in array.array)
            state.name.extend([ACTUATOR_JOINT, TRIGGER_JOINT])
            state.position.extend([self.sim.rack, self.sim.trigger])
            state.velocity.extend([0.0, 0.0])
            state.effort.extend([0.0, 0.0])
            self.ready_pub.publish(Bool(data=not emergency_stop))
        self.joint_state_pub.publish(state)
        self.estop_pub.publish(Bool(data=emergency_stop))

    def _record(
        self, ground_truth_masks: dict[str, np.ndarray], emergency_stop: bool
    ) -> None:
        target_mask = ground_truth_masks.get(self.sim.target_id)
        target_visible = bool(target_mask is not None and np.any(target_mask))
        detected = bool(self.latest["confidence"] > 0.0)
        predicted_mask = self.latest["mask"]
        iou = 0.0
        if detected and target_mask is not None and predicted_mask is not None:
            iou = self.sim._mask_iou(predicted_mask, target_mask)
        locked = bool(self.latest["locked"] and detected and target_visible)
        if locked and iou <= 0.05:
            self.false_lock_count += 1
        if not detected and self.was_detected and self.occlusion_started is None:
            self.occlusion_started = self.current_t
        if detected and self.occlusion_started is not None:
            self.reacquire_times.append(self.current_t - self.occlusion_started)
            self.occlusion_started = None
        self.was_detected = detected

        projection = self.sim.project(self.sim.target_object, self.current_t)
        if projection is None:
            ground_angular = [0.0, 0.0]
        else:
            ground_angular = [float(projection[4]), float(projection[5])]
        pixel_norm = math.hypot(
            math.tan(ground_angular[0]) * self.sim.fx,
            math.tan(ground_angular[1]) * self.sim.fy,
        )
        angular_norm = math.hypot(*ground_angular)
        target_mask_topic = None
        if detected:
            target_mask_topic = {
                "id": self.sim.target_id,
                "area_px": int((predicted_mask > 0).sum()) if predicted_mask is not None else 0,
                "iou_vs_sim": iou,
            }
        topics = self.scenario["topics"]
        sample = {
            "frame_index": self.frame_index,
            "t_s": self.current_t,
            "topics": {
                topics["camera_image"]: {
                    "width": self.sim.width,
                    "height": self.sim.height,
                    "encoding": "bgr8",
                    "frame_id": self.scenario["camera"]["frame_id"],
                    "dropped": self.sim.camera_frame_dropped(self.current_t),
                },
                topics["camera_info"]: {
                    "width": self.sim.width,
                    "height": self.sim.height,
                    "k": [self.sim.fx, 0.0, self.sim.cx, 0.0, self.sim.fy, self.sim.cy, 0.0, 0.0, 1.0],
                },
                topics["segments"]: self.latest["segments"],
                topics["target_mask"]: target_mask_topic,
                topics["target_bbox"]: self.latest["bbox"] or None,
                topics["target_confidence"]: self.latest["confidence"],
                topics["pixel_error"]: self.latest["pixel_error"],
                topics["angular_error"]: self.latest["angular_error"],
                topics["selected_id"]: self.latest["selected_id"],
                topics["locked"]: locked,
                topics["joint_commands"]: {
                    "name": ["pan_joint", "tilt_joint"],
                    "position": [self.sim.pan_cmd, self.sim.tilt_cmd],
                },
                topics["joint_states"]: {
                    "name": ["pan_joint", "tilt_joint"],
                    "position": [self.sim.pan, self.sim.tilt],
                },
                topics["diagnostics"]: {
                    "backend": "pantiltros:ultralytics",
                    "yolo_latency_ms": self.latest["latency_ms"],
                },
                topics["emergency_stop"]: emergency_stop,
            },
            "derived": {
                "target_visible": target_visible,
                "target_predicted": False,
                "camera_frame_dropped": self.sim.camera_frame_dropped(self.current_t),
                "pixel_error_norm_px": pixel_norm,
                "angular_error_norm_rad": angular_norm,
                "target_iou": iou,
                "target_confidence": self.latest["confidence"],
                "yolo_latency_ms": self.latest["latency_ms"],
            },
        }
        self.records.append(sample)

    def _report(self) -> dict[str, Any]:
        ignore_initial = float(self.scenario["scoring"]["ignore_initial_s"])
        scored = [sample for sample in self.records if sample["t_s"] >= ignore_initial]
        visible = [sample for sample in scored if sample["derived"]["target_visible"]]
        detected = [
            sample
            for sample in visible
            if sample["derived"]["target_confidence"] > 0.0
        ]
        locked = [
            sample
            for sample in visible
            if sample["topics"][self.scenario["topics"]["locked"]]
        ]
        pixel_errors = [sample["derived"]["pixel_error_norm_px"] for sample in visible]
        angular_errors = [
            math.degrees(sample["derived"]["angular_error_norm_rad"])
            for sample in visible
        ]
        ious = [sample["derived"]["target_iou"] for sample in detected]
        latencies = [sample["derived"]["yolo_latency_ms"] for sample in scored]
        scored_commands = [
            saturated for t_s, saturated in self.command_log if t_s >= ignore_initial
        ]
        summary = {
            "sample_count": len(self.records),
            "scored_sample_count": len(scored),
            "target_visible_sample_count": len(visible),
            "lock_fraction": len(locked) / max(1, len(visible)),
            "target_detection_fraction": len(detected) / max(1, len(visible)),
            "mean_pixel_error_px": sum(pixel_errors) / max(1, len(pixel_errors)),
            "p95_pixel_error_px": percentile(pixel_errors, 95),
            "mean_angular_error_deg": sum(angular_errors) / max(1, len(angular_errors)),
            "p95_angular_error_deg": percentile(angular_errors, 95),
            "target_mean_iou": sum(ious) / max(1, len(ious)),
            "target_min_iou": min(ious) if ious else 0.0,
            "false_lock_count": self.false_lock_count,
            "max_reacquire_time_s": max(self.reacquire_times) if self.reacquire_times else 0.0,
            "command_saturation_fraction": (
                sum(scored_commands) / len(scored_commands) if scored_commands else 0.0
            ),
            "dropped_command_count": self.dropped_command_count,
            "joint_limit_violation_count": self.sim.joint_limit_violations,
            "emergency_stop_violation_count": self.sim.emergency_stop_violations,
            "max_yolo_latency_ms": max(latencies) if latencies else 0.0,
            "topic_contract": self.scenario["topics"],
            "yolo_contract": self.scenario["yolo"],
        }
        report = {
            "scenario": self.scenario["name"],
            "variant": self.variant,
            "target_id": self.sim.target_id,
            "detector": {"backend": "pantiltros:ultralytics"},
            "summary": summary,
            "samples": self.records,
        }
        aligned = list(self.frame_results.values())
        visible_results = [r for r in aligned if r['target_visible'] and r['t_s'] >= ignore_initial]
        overlaps = [r for r in visible_results if r['iou'] > .05]
        report['frame_delivery'] = {
            'startup_discovery_wait_s': self.discovery_wait_s,
            'rendered_frames': self.frame_index,
            'injected_camera_drops': self.frame_index - len(self.input_frames),
            'published_frames': len(self.input_frames),
            'processed_unique_frames': len(self.frame_results),
            'frames_without_result': len(self.input_frames) - len(self.frame_results),
            'visible_processed_frames': len(visible_results),
            'visible_frames_without_mask': sum(not r['mask_present'] for r in visible_results),
            'visible_frames_wrong_mask': sum(r['mask_present'] and r['iou'] <= .05 for r in visible_results),
            'target_overlap_fraction': len(overlaps) / max(1, len(visible_results)),
            'mean_iou_when_overlapping': sum(r['iou'] for r in overlaps) / max(1, len(overlaps)),
            'capture_to_result_p95_ms': percentile([r['capture_to_result_ms'] for r in aligned], 95),
        }
        if self.actuator_fitted:
            report["launcher"] = self._launcher_report()
        return report

    def _record_launcher_truth(self, masks: dict[str, np.ndarray]) -> None:
        """Match the active track to scene truth after automatic target changes."""
        track_id = (self.status or {}).get("target_id")
        detection = next((d for d in self.latest["segments"] if d["id"] == track_id), None)
        if detection is None:
            return
        a = detection["bbox_xyxy"]
        matches = []
        for obj in self.scenario["objects"]:
            if obj["yolo_prompt"] != detection["class"] or obj["id"] not in masks:
                continue
            ys, xs = np.nonzero(masks[obj["id"]])
            if not len(xs):
                continue
            b = (xs.min(), ys.min(), xs.max() + 1, ys.max() + 1)
            intersection = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(
                0., min(a[3], b[3]) - max(a[1], b[1]))
            union = ((a[2] - a[0]) * (a[3] - a[1])
                     + (b[2] - b[0]) * (b[3] - b[1]) - intersection)
            if union > 0 and intersection / union > .25:
                matches.append((intersection / union, obj))
        if not matches:
            return
        _, obj = max(matches, key=lambda item: item[0])
        projection = self.sim.project(obj, self.current_t)
        if projection is not None:
            self.launcher_truth[self.current_t] = {
                "target_id": track_id, "scene_object_id": obj["id"],
                "true_aim_error_deg": math.degrees(math.hypot(*projection[4:6])),
            }

    def _launcher_report(self) -> dict[str, Any]:
        """The presses, the shots with their cycle times and the true aim
        error at each, and any priming or firing command that no accepted
        press explains."""
        log = self.launcher_log
        accepted = [entry["t_s"] for entry in self.last_tags
                    if entry["tag"].get("result") == "running"]
        window = 15.0  # a cycle (prime, chamber, trigger) ends well within this
        unexplained = [t for t in self.priming_commands + self.firing_commands
                       if not any(start - 0.5 <= t <= start + window for start in accepted)]
        shots = []
        for shot in self.sim.shots:
            press = max((t for t in accepted if t <= shot), default=None)
            ready = next((t for t, _, _, state in log
                          if t > shot and state in ("ready", "empty")), None)
            shots.append({
                "t_s": shot,
                "tag_at_s": press,
                "press_to_shot_s": None if press is None else round(shot - press, 3),
                "press_to_ready_s": (None if press is None or ready is None
                                     else round(ready - press, 3)),
                **self.launcher_truth.get(round(shot, 6), {
                    "target_id": None, "scene_object_id": None, "true_aim_error_deg": None,
                }),
            })
        phases: list[list[Any]] = []
        for t, _, _, state in log:
            if state and (not phases or phases[-1][1] != state):
                phases.append([t, state])
        return {
            "locks": self.locks,
            "presses": self.presses,
            "acknowledged_faults": self.acknowledged,
            "last_tag_changes": self.last_tags,
            "phases": phases,
            "shots": shots,
            "max_rack_m": max((rack for _, rack, _, _ in log), default=0.0),
            "max_trigger_rad": max((trigger for _, _, trigger, _ in log), default=0.0),
            "priming_command_count": len(self.priming_commands),
            "firing_command_count": len(self.firing_commands),
            "unexplained_command_count": len(unexplained),
            "final_rack_m": log[-1][1] if log else 0.0,
            "final_trigger_rad": log[-1][2] if log else 0.0,
        }

    def _finish(self) -> None:
        self.finished = True
        report = self._report()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report["summary"], indent=2), flush=True)
        if "launcher" in report:
            launcher = {key: value for key, value in report["launcher"].items()
                        if key not in ("last_tag_changes", "phases")}
            print(json.dumps({"launcher": launcher}, indent=2), flush=True)
        self.get_logger().info(f"Wrote turret metrics to {self.output}")
        self.timer.cancel()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario", default="applications/turret_sim/scenarios/warehouse_tracking.json"
    )
    parser.add_argument(
        "--output", default="generated/turret/turret_tracking_metrics.json"
    )
    parser.add_argument("--variant", default="nominal")
    parser.add_argument("--save-frames")
    parser.add_argument(
        "--actuator", type=str2bool, default=False,
        help="fit the dart launcher (true/false)",
    )
    parser.add_argument(
        "--tag-at", type=float,
        help="from this time (s) on, reload and press Fire when /turret/status enables it",
    )
    parser.add_argument("--tag-attempts", type=int, default=10)
    parser.add_argument(
        "--tag-lock", type=str2bool, default=False,
        help="tap the target's box (lock by ID) before each Fire (true/false)",
    )
    parser.add_argument("--darts", type=int, default=3, help="the operator's reload")
    args, ros_args = parser.parse_known_args()
    scenario = json.loads(Path(args.scenario).read_text())
    rclpy.init(args=ros_args)
    node = TurretRosSimulator(
        scenario,
        Path(args.output),
        args.variant,
        Path(args.save_frames) if args.save_frames else None,
        actuator=args.actuator,
        tag_at=args.tag_at if args.actuator else None,
        tag_attempts=args.tag_attempts,
        tag_lock=args.tag_lock,
        darts=args.darts,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

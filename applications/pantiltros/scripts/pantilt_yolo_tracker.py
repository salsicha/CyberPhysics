#!/usr/bin/env python3
"""Shared ROS 2 YOLO perception and closed-loop control for PanTiltROS."""

from __future__ import annotations

import json
import math
import os
import time
from bisect import bisect_right
from collections import deque
from pathlib import Path
from typing import Any

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
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from ultralytics import YOLO


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

    def reference(self, t_image: float, now: float) -> tuple[tuple[float, float], str]:
        """Turret position when the frame stamped ``t_image`` was captured."""
        if self.has_feedback(now):
            return self._lookup(self.measured, t_image, True), "measured"
        return (
            self._lookup(self.commanded, t_image - self.actuator_latency_s, False),
            "commanded",
        )

    def update(
        self, t_image: float, err_pan: float, err_tilt: float, now: float
    ) -> tuple[float, float] | None:
        """Return the next (pan, tilt) command, or None for invalid input."""
        if not finite(t_image, err_pan, err_tilt, now):
            return None
        reference, _ = self.reference(t_image, now)
        target = (reference[0] + err_pan, reference[1] + err_tilt)
        if (
            self.last_command is not None
            and 0.0 <= now - self.last_command[0] <= self.resume_timeout_s
        ):
            previous = self.last_command[1:]
            dt = max(1e-3, now - self.last_command[0])
        else:
            # First command, or resuming after a pause: start from the best
            # estimate of where the turret is now rather than a stale command.
            previous = (
                self.measured[-1][1:] if self.has_feedback(now) else self.commanded[-1][1:]
            )
            dt = min(self.resume_timeout_s, 0.1)
        step_limit = self.max_rate * dt
        command = []
        for prev, goal, (low, high) in zip(previous, target, self.limits):
            step = max(-step_limit, min(step_limit, self.gain * (goal - prev)))
            command.append(clamp(prev + step, low, high))
        self.last_command = (float(now), command[0], command[1])
        self.commanded.append(self.last_command)
        self._prune(self.commanded, now)
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


class PanTiltYoloTracker(Node):
    def __init__(self) -> None:
        super().__init__("pantilt_yolo_tracker")
        defaults: dict[str, Any] = {
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
            "model_path": "/workspace/cache/models/yoloe-26s-seg.pt",
            "device": "0",
            "image_size": 640,
            "confidence": 0.003,
            "class_prompts": ["blue utility cart"],
            "target_id": "cart_blue_01",
            "target_class": "blue utility cart",
            "initial_selection_normalized": [0.13, 0.565],
            "association_gate_fraction": 0.50,
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
            "lock_tolerance_px": 36.0,
            "publish_annotated_image": True,
            "show_window": False,
            "window_name": "PanTiltROS YOLO",
            "warmup": True,
            "ready_file": "/tmp/pantilt_yolo_ready",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

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
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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
        if stamp <= 0.0 or abs(stamp - now) > 2.0:
            stamp = now
        if not self.controller.add_measured(stamp, pan, tilt):
            self.get_logger().warning(
                "ignoring non-finite joint state", throttle_duration_sec=5.0
            )

    def _emergency_stop(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        if engaged != self.estop_engaged:
            self.get_logger().warning(
                "emergency stop " + ("engaged" if engaged else "released")
            )
        self.estop_engaged = engaged

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
        if self.target_centroid is None:
            self.target_centroid = (
                self.initial_selection[0] * width,
                self.initial_selection[1] * height,
            )

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
        target = self._associate_target(detections, width, height)
        self._publish_outputs(
            msg, frame, target, detections, latency_ms, width, height, capture_time
        )
        self.frame_count += 1

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
                    "class": str(result.names[int(class_index)]),
                    "confidence": float(confidence),
                    "bbox_xywh": [
                        (x1 + x2) * 0.5,
                        (y1 + y2) * 0.5,
                        x2 - x1,
                        y2 - y1,
                    ],
                    "centroid_px": [centroid[0], centroid[1]],
                    "mask_area_px": int((mask > 0).sum()),
                    "mask": mask,
                }
            )
        return detections

    def _associate_target(
        self, detections: list[dict[str, Any]], width: int, height: int
    ) -> dict[str, Any] | None:
        candidates = [
            detection
            for detection in detections
            if detection["class"] in (self.target_class, self.target_id)
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
        self.target_centroid = tuple(target["centroid_px"])
        return target

    def _publish_outputs(
        self,
        source: Image,
        frame: np.ndarray,
        target: dict[str, Any] | None,
        detections: list[dict[str, Any]],
        latency_ms: float,
        width: int,
        height: int,
        capture_time: float,
    ) -> None:
        public_detections = [
            {key: value for key, value in detection.items() if key != "mask"}
            for detection in detections
        ]
        self.segments_pub.publish(String(data=json.dumps(public_detections)))
        fx, fy, cx, cy = self._intrinsics(width, height)
        pixel_x = pixel_y = angular_x = angular_y = confidence = 0.0
        locked = False
        selected_id = ""
        bbox: list[float] = []
        mask = np.zeros((height, width), dtype=np.uint8)
        if target is not None:
            pixel_x = float(target["centroid_px"][0]) - cx
            pixel_y = float(target["centroid_px"][1]) - cy
            angular_x = math.atan2(pixel_x, fx)
            angular_y = -math.atan2(pixel_y, fy)
            confidence = float(target["confidence"])
            bbox = [float(value) for value in target["bbox_xywh"]]
            mask = target["mask"]
            selected_id = self.target_id
            locked = math.hypot(pixel_x, pixel_y) <= self.lock_tolerance
            self._publish_command(angular_x, angular_y, capture_time)

        self.mask_pub.publish(mask_message(mask, source))
        self.bbox_pub.publish(Float32MultiArray(data=bbox))
        self.confidence_pub.publish(Float32(data=confidence))
        self.pixel_error_pub.publish(Float32MultiArray(data=[pixel_x, pixel_y]))
        self.angular_error_pub.publish(
            Float32MultiArray(data=[angular_x, angular_y])
        )
        self.selected_id_pub.publish(String(data=selected_id))
        self.locked_pub.publish(Bool(data=locked))
        if self.publish_annotated_image or self.show_window:
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
                f"{label_prefix}{detection['class']} "
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

    def _publish_command(
        self, angular_x: float, angular_y: float, capture_time: float
    ) -> None:
        now = self._now()
        self.command_state = self._command_gate(now)
        if self.command_state not in ("measured", "commanded"):
            return
        command_position = self.controller.update(
            capture_time, angular_x, angular_y, now
        )
        if command_position is None:
            self.command_state = "invalid_error"
            return
        pan, tilt = command_position
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["pan_joint", "tilt_joint"]
        command.position = [pan, tilt]
        self.command_pub.publish(command)

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
            KeyValue(key="emergency_stop", value=str(self.estop_engaged).lower()),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> bool:
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

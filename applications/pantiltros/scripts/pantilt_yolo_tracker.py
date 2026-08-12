#!/usr/bin/env python3
"""Shared ROS 2 YOLO perception and closed-loop control for PanTiltROS."""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from ultralytics import YOLO


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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


def joint_position(msg: JointState | None, name: str, default: float = 0.0) -> float:
    if msg is not None and name in msg.name:
        index = msg.name.index(name)
        if index < len(msg.position):
            return float(msg.position[index])
    return default


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
            "selected_id_topic": "/turret/target/selected_id",
            "locked_topic": "/turret/target/locked",
            "diagnostics_topic": "/turret/diagnostics",
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
            "kp": 0.82,
            "kd": 0.08,
            "pan_min_rad": -1.5708,
            "pan_max_rad": 1.5708,
            "tilt_min_rad": -0.65,
            "tilt_max_rad": 0.75,
            "lock_tolerance_px": 36.0,
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
        self.kp = float(self.parameter("kp"))
        self.kd = float(self.parameter("kd"))
        self.pan_min = float(self.parameter("pan_min_rad"))
        self.pan_max = float(self.parameter("pan_max_rad"))
        self.tilt_min = float(self.parameter("tilt_min_rad"))
        self.tilt_max = float(self.parameter("tilt_max_rad"))
        self.lock_tolerance = float(self.parameter("lock_tolerance_px"))
        self.ready_file = Path(str(self.parameter("ready_file")))

        self.camera_info: CameraInfo | None = None
        self.joint_state: JointState | None = None
        self.last_command = (0.0, 0.0)
        self.last_error: tuple[float, float] | None = None
        self.last_frame_time: float | None = None
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

    def _joint_state(self, msg: JointState) -> None:
        self.joint_state = msg

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
        self._publish_outputs(msg, target, detections, latency_ms, width, height)
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
        target: dict[str, Any] | None,
        detections: list[dict[str, Any]],
        latency_ms: float,
        width: int,
        height: int,
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
            self._publish_command(angular_x, angular_y)
        else:
            self.last_error = None

        self.mask_pub.publish(mask_message(mask, source))
        self.bbox_pub.publish(Float32MultiArray(data=bbox))
        self.confidence_pub.publish(Float32(data=confidence))
        self.pixel_error_pub.publish(Float32MultiArray(data=[pixel_x, pixel_y]))
        self.angular_error_pub.publish(
            Float32MultiArray(data=[angular_x, angular_y])
        )
        self.selected_id_pub.publish(String(data=selected_id))
        self.locked_pub.publish(Bool(data=locked))
        self._publish_diagnostics(latency_ms, len(detections), target is not None)

    def _publish_command(self, angular_x: float, angular_y: float) -> None:
        if not self.command_enabled:
            return
        now = time.monotonic()
        dt = 0.0 if self.last_frame_time is None else max(1e-4, now - self.last_frame_time)
        derivative_x = derivative_y = 0.0
        if self.last_error is not None and dt > 0.0:
            derivative_x = (angular_x - self.last_error[0]) / dt
            derivative_y = (angular_y - self.last_error[1]) / dt
        self.last_frame_time = now
        self.last_error = (angular_x, angular_y)
        pan = joint_position(self.joint_state, "pan_joint", self.last_command[0])
        tilt = joint_position(self.joint_state, "tilt_joint", self.last_command[1])
        pan = clamp(pan + self.kp * angular_x + self.kd * derivative_x * dt, self.pan_min, self.pan_max)
        tilt = clamp(tilt + self.kp * angular_y + self.kd * derivative_y * dt, self.tilt_min, self.tilt_max)
        self.last_command = (pan, tilt)
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
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> bool:
        try:
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

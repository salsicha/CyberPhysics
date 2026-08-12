#!/usr/bin/env python3
"""ROS 2 camera and actuator simulator for the shared PanTiltROS YOLO node."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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


def mono_image(msg: Image) -> np.ndarray | None:
    if msg.encoding != "mono8" or msg.step < msg.width:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        return None
    return raw[:expected].reshape(int(msg.height), int(msg.step))[:, : int(msg.width)].copy()


class TurretRosSimulator(Node):
    def __init__(
        self,
        scenario: dict[str, Any],
        output: Path,
        variant: str,
        save_frames: Path | None,
    ) -> None:
        super().__init__("turret_ros_sim")
        self.scenario = scenario
        self.output = output
        self.variant = variant
        self.save_frames = save_frames
        if save_frames is not None:
            save_frames.mkdir(parents=True, exist_ok=True)

        self.sim = TurretSim(scenario, variant_name=variant)
        self.sim.cv2 = cv2
        self.sim.np = np
        self.sim.background = self.sim._render_background()
        self.dt = float(scenario["dt_s"])
        self.steps = int(float(scenario["duration_s"]) / self.dt) + 1
        self.frame_index = 0
        self.current_t = 0.0
        self.finished = False
        self.records: list[dict[str, Any]] = []
        self.reacquire_times: list[float] = []
        self.occlusion_started: float | None = None
        self.was_detected = False
        self.false_lock_count = 0

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
        self.estop_pub = self.create_publisher(Bool, topics["emergency_stop"], 10)
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
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"Turret ROS simulation ready: {self.steps} frames at {1.0 / self.dt:.1f} Hz"
        )

    def _joint_command(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if "pan_joint" not in positions or "tilt_joint" not in positions:
            return
        pan_limits = self.scenario["pantilt"]["pan_limit_rad"]
        tilt_limits = self.scenario["pantilt"]["tilt_limit_rad"]
        pan = max(float(pan_limits[0]), min(float(pan_limits[1]), float(positions["pan_joint"])))
        tilt = max(float(tilt_limits[0]), min(float(tilt_limits[1]), float(positions["tilt_joint"])))
        execute_at = self.current_t + float(self.scenario["pantilt"]["command_latency_s"])
        self.sim.command_queue.append((execute_at, pan, tilt))

    def _segments(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            self.latest["segments"] = value if isinstance(value, list) else []
        except json.JSONDecodeError:
            self.latest["segments"] = []

    def _mask(self, msg: Image) -> None:
        self.latest["mask"] = mono_image(msg)

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
        if self.frame_index >= self.steps:
            self._finish()
            return
        self.current_t = round(self.frame_index * self.dt, 6)
        emergency_stop = self.sim.in_emergency_stop(self.current_t)
        self.sim.update_servo(self.current_t, self.dt, emergency_stop)
        frame, ground_truth_masks = self.sim.render_frame(self.current_t)
        self._record(ground_truth_masks, emergency_stop)
        stamp = self.get_clock().now().to_msg()
        self._publish_state(stamp, emergency_stop)
        if not self.sim.camera_frame_dropped(self.current_t):
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
            "command_saturation_fraction": 0.0,
            "joint_limit_violation_count": self.sim.joint_limit_violations,
            "emergency_stop_violation_count": self.sim.emergency_stop_violations,
            "max_yolo_latency_ms": max(latencies) if latencies else 0.0,
            "topic_contract": self.scenario["topics"],
            "yolo_contract": self.scenario["yolo"],
        }
        return {
            "scenario": self.scenario["name"],
            "variant": self.variant,
            "target_id": self.sim.target_id,
            "detector": {"backend": "pantiltros:ultralytics"},
            "summary": summary,
            "samples": self.records,
        }

    def _finish(self) -> None:
        self.finished = True
        report = self._report()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report["summary"], indent=2), flush=True)
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
    args, ros_args = parser.parse_known_args()
    scenario = json.loads(Path(args.scenario).read_text())
    rclpy.init(args=ros_args)
    node = TurretRosSimulator(
        scenario,
        Path(args.output),
        args.variant,
        Path(args.save_frames) if args.save_frames else None,
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

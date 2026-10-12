"""Isaac Sim scene and deterministic grasp semantics for the SO-101 demo."""

import json
import math
import os
import re
import tempfile
import time
from pathlib import Path

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics
from score_picking_task import score
from so101_common import (
    JOINT_LIMIT_VIOLATION_TOLERANCE,
    LOWER_LIMITS,
    SCENE_COLORS,
    UPPER_LIMITS,
    forward_kinematics,
    scene_color,
    scene_support_errors,
)

# The palette every simulator renders the scene with (so101_common).
COLORS = SCENE_COLORS


def write_json_atomic(path, payload):
    """Replace complete reports, including files created by another container UID."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=path.parent, prefix=f".{path.name}.", delete=False
        ) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def load_scenario(path):
    if not path:
        return {}
    return json.loads(Path(path).read_text())


def selected_task(scenario, task_id=""):
    tasks = scenario.get("tasks", [])
    if task_id:
        for task in tasks:
            if task.get("id") == task_id:
                return task
        raise ValueError(f"Task {task_id!r} not found in scenario")
    if not tasks:
        raise ValueError("Scenario contains no tasks")
    return tasks[0]


def _safe_name(value):
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


def _color(entry):
    return scene_color(entry)


def _rotation_degrees(entry):
    rpy = entry.get("rpy", [0.0, 0.0, 0.0])
    return Gf.Vec3f(*(math.degrees(float(value)) for value in rpy[:3]))


def create_box(stage, path, entry, kinematic=False, collision=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*_color(entry))])
    xform = UsdGeom.Xformable(cube)
    translate = xform.AddTranslateOp()
    translate.Set(Gf.Vec3d(*entry.get("xyz", [0.0, 0.0, 0.0])[:3]))
    xform.AddRotateXYZOp().Set(_rotation_degrees(entry))
    xform.AddScaleOp().Set(Gf.Vec3f(*entry.get("size", [0.05, 0.05, 0.05])[:3]))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if kinematic:
        body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        body.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(float(entry.get("mass_kg", 0.04)))
    return translate


def create_cylinder(stage, path, entry, kinematic=False, collision=True):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(float(entry.get("radius_m", 0.025)))
    cylinder.CreateHeightAttr(float(entry.get("height_m", 0.05)))
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(*_color(entry))])
    xform = UsdGeom.Xformable(cylinder)
    translate = xform.AddTranslateOp()
    translate.Set(Gf.Vec3d(*entry.get("xyz", [0.0, 0.0, 0.0])[:3]))
    xform.AddRotateXYZOp().Set(_rotation_degrees(entry))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    if kinematic:
        body = UsdPhysics.RigidBodyAPI.Apply(cylinder.GetPrim())
        body.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(cylinder.GetPrim()).CreateMassAttr(float(entry.get("mass_kg", 0.04)))
    return translate


class SO101TaskScene:
    """Materialize scenario assets, track grasp state, and emit scored telemetry."""

    def __init__(self, stage, scenario, task_id="", telemetry_path=""):
        self.stage = stage
        self.scenario = scenario
        self.task = selected_task(scenario, task_id)
        self.task_id = self.task["id"]
        self.robot_spawn = np.asarray(
            scenario.get("robot_spawn", {}).get("xyz", [0.0, 0.0, 0.0]), dtype=np.float64
        )
        self.objects = {entry["id"]: entry for entry in scenario.get("objects", [])}
        self.assets = {entry["name"]: entry for entry in scenario.get("static_assets", [])}
        self.target = self.objects[self.task["target_object"]]
        self.destination = self.assets[self.task["destination"]]
        self.target_initial = np.asarray(self.target["xyz"][:3], dtype=np.float64)
        self.target_position = self.target_initial.copy()
        self.destination_position = np.asarray(self.destination["xyz"][:3], dtype=np.float64)
        self.target_translate = None
        self.attached = False
        # Where the target sits relative to the jaw centre while held: the
        # jaws close on it wherever it is (the demo grasps above the centre of
        # an object standing on the table, so the jaw tips clear the table),
        # and it keeps that offset until released.
        self.grasp_offset = np.zeros(3)
        self.released = False
        self.grasp_closed = False
        self.max_lift = 0.0
        self.status = "waiting_for_policy"
        self.started_at = None
        self.last_sample_at = 0.0
        self.last_flush_at = 0.0
        self.last_printed_status = None
        self.reported_success = False
        self.samples = []
        # Measured here or reported per command by the policy bridge. None
        # means "not measured", never "zero".
        self.joint_limit_violations = 0
        self.policy_command_count = 0
        self.saturated_command_count = 0
        self.max_policy_latency_ms = None
        self.max_observation_age_ms = None
        self.policy_used_ground_truth = None
        self.telemetry_path = Path(telemetry_path) if telemetry_path else None
        self.telemetry_error_reported = False
        self.metrics_path = (
            self.telemetry_path.with_name(self.telemetry_path.stem + "_metrics.json")
            if self.telemetry_path else None
        )
        self._build_scene()

    def _build_scene(self):
        # Objects are kinematic here (they stay where the scenario puts them),
        # so a scenario whose objects do not rest on their surfaces would
        # start Isaac in a different state from Gazebo, which drops them.
        for error in scene_support_errors(self.scenario):
            print(f"SO-101 scenario inconsistency: {error}", flush=True)
        UsdGeom.Xform.Define(self.stage, "/World/Task")
        for asset in self.scenario.get("static_assets", []):
            create_box(self.stage, f"/World/Task/{_safe_name(asset['name'])}", asset)
        for entry in self.scenario.get("objects", []):
            path = f"/World/Task/{_safe_name(entry['id'])}"
            is_target = entry["id"] == self.target["id"]
            # The deterministic demo attaches the target kinematically once
            # the jaws close. A second kinematic collision body on that same
            # target blocks the arm before attachment can occur.
            if entry.get("shape") == "cylinder":
                translate = create_cylinder(
                    self.stage, path, entry, kinematic=True, collision=not is_target
                )
            else:
                translate = create_box(
                    self.stage, path, entry, kinematic=True, collision=not is_target
                )
            if is_target:
                self.target_translate = translate

    def _set_target_position(self, position):
        self.target_position = np.asarray(position, dtype=np.float64)
        if self.target_translate is not None:
            self.target_translate.Set(Gf.Vec3d(*self.target_position.tolist()))

    def _elapsed(self, now):
        return 0.0 if self.started_at is None else max(0.0, now - self.started_at)

    def record_policy_command(self, stats):
        """Fold one bridge command report (see groot_so101_bridge) into the metrics."""
        if not isinstance(stats, dict):
            return

        def finite(value):
            return (isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) and value >= 0.0)

        self.policy_command_count += 1
        if stats.get("command_saturated") is True:
            self.saturated_command_count += 1
        latency = stats.get("policy_latency_ms")
        if finite(latency):
            self.max_policy_latency_ms = max(self.max_policy_latency_ms or 0.0, float(latency))
        age = stats.get("observation_age_ms")
        if finite(age):
            self.max_observation_age_ms = max(self.max_observation_age_ms or 0.0, float(age))
        used = stats.get("policy_used_ground_truth")
        if isinstance(used, bool):
            self.policy_used_ground_truth = bool(self.policy_used_ground_truth) or used

    def update(self, joints, command_received, now=None):
        now = time.monotonic() if now is None else float(now)
        joints = np.asarray(joints, dtype=np.float64)
        end_effector = forward_kinematics(joints, self.robot_spawn)
        if self.started_at is None and command_received:
            self.started_at = now
            self.status = "approaching_target"

        if self.started_at is not None:
            if (np.any(joints < LOWER_LIMITS - JOINT_LIMIT_VIOLATION_TOLERANCE)
                    or np.any(joints > UPPER_LIMITS + JOINT_LIMIT_VIOLATION_TOLERANCE)):
                self.joint_limit_violations += 1
            distance = float(np.linalg.norm(end_effector - self.target_position))
            if not self.attached and not self.released and joints[5] <= 0.018 and distance <= 0.045:
                self.attached = True
                self.grasp_closed = True
                self.grasp_offset = self.target_position - end_effector
                self.status = "target_grasped"
            if self.attached:
                self._set_target_position(end_effector + self.grasp_offset)
                self.max_lift = max(self.max_lift, float(self.target_position[2] - self.target_initial[2]))
                if self.max_lift >= float(self.task.get("success", {}).get("object_lift_height_m", 0.08)):
                    self.status = "transporting_target"
                if joints[5] >= 0.03:
                    self.attached = False
                    self.released = True
                    self.status = "target_released"

            if now - self.last_sample_at >= 0.05 or self.status == "solved":
                self.samples.append({
                    "time": self._elapsed(now),
                    "end_effector_xyz": end_effector.tolist(),
                    "object_xyz": self.target_position.tolist(),
                    "gripper_width_m": float(joints[5]),
                    "attached": self.attached,
                })
                self.last_sample_at = now
                if self.released:
                    metrics = score(self.scenario, self.telemetry(now), self.task_id)
                    # "solved" is the physical outcome; whether the run is a
                    # valid policy evaluation is metrics["success"].
                    self.status = "solved" if metrics["task_completed"] else "target_released"

        if self.status != self.last_printed_status:
            print(f"SO-101 task status: {self.status}", flush=True)
            self.last_printed_status = self.status
            self._flush(now)
        elif self.started_at is not None and now - self.last_flush_at >= 1.0:
            self._flush(now)
        return self.status

    def telemetry(self, now=None):
        now = time.monotonic() if now is None else float(now)
        return {
            "task_id": self.task_id,
            "status": self.status,
            "duration_s": self._elapsed(now),
            "grasp_closed": self.grasp_closed,
            # Reported by the policy bridge; None until a command report arrives.
            "policy_used_ground_truth": self.policy_used_ground_truth,
            # This scene does not register contact reporting, so collisions are
            # unmeasured (None), not zero.
            "collisions": None,
            "failed_grasps": [],
            "joint_limit_violations": self.joint_limit_violations,
            "policy_commands": None if not self.policy_command_count else {
                "count": self.policy_command_count,
                "saturated": self.saturated_command_count,
                "max_policy_latency_ms": self.max_policy_latency_ms,
                "max_observation_age_ms": self.max_observation_age_ms,
            },
            "samples": self.samples,
        }

    def summary(self, now=None):
        return {
            "task_id": self.task_id,
            "status": self.status,
            "duration_s": self._elapsed(time.monotonic() if now is None else now),
            "attached": self.attached,
            "max_lift_height_m": self.max_lift,
            "object_xyz": self.target_position.tolist(),
            "destination_xyz": self.destination_position.tolist(),
        }

    def close(self):
        if self.started_at is not None:
            now = time.monotonic()
            self._flush(now)
            # Also report runs that never solved the task, so acceptance
            # statistics are not computed from successful runs only.
            if self.metrics_path is not None and not self.reported_success:
                self._write_metrics(self.telemetry(now))

    def _flush(self, now):
        if self.telemetry_path is None:
            return
        payload = self.telemetry(now)
        try:
            write_json_atomic(self.telemetry_path, payload)
        except OSError as exc:
            self._disable_telemetry(exc)
            return
        self.last_flush_at = now
        if self.status == "solved" and self.metrics_path is not None and not self.reported_success:
            metrics = self._write_metrics(payload)
            if metrics is None:
                return
            self.reported_success = True
            print(
                f"SO-101 TASK SOLVED: success={metrics['success']} "
                f"place_error={metrics['final_place_error_m']:.4f}m "
                f"duration={metrics['duration_s']:.2f}s "
                f"unmeasured={metrics['unmeasured_checks']}",
                flush=True,
            )

    def _write_metrics(self, payload):
        metrics = score(self.scenario, payload, self.task_id)
        try:
            write_json_atomic(self.metrics_path, metrics)
        except OSError as exc:
            self._disable_telemetry(exc)
            return None
        return metrics

    def _disable_telemetry(self, exc):
        path = self.telemetry_path
        if not self.telemetry_error_reported:
            print(
                f"SO-101 telemetry disabled after write failure at {path}: {exc}",
                flush=True,
            )
            self.telemetry_error_reported = True
        self.telemetry_path = None
        self.metrics_path = None

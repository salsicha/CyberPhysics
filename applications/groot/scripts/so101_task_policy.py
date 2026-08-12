#!/usr/bin/env python3
"""Deterministic SO-101 tabletop task plan used by the GR00T demo policy.

The controller deliberately consumes only the fixed benchmark specification and
joint-state observations. It does not consume simulator object poses. Real
visual-policy evaluation remains available through ``SO101_GROOT_MODE=real``.
"""

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


JOINT_LIMITS = np.array([
    [-1.9199, 1.9199],
    [-1.7453, 1.7453],
    [-1.7453, 1.7453],
    [-1.6581, 1.6581],
    [-2.7439, 2.7439],
    [0.0, 0.04],
], dtype=np.float32)

# Kinematic dimensions from systems/so101/urdf/so101.urdf. The last segment
# ends at the centre of the jaws, which is the point used for demo grasping.
SHOULDER_PIVOT_Z = 0.0225 + 0.065 + 0.045
ARM_LINK_LENGTHS = np.array([0.14, 0.13, 0.055 + 0.052 + 0.035], dtype=np.float64)


@dataclass(frozen=True)
class TaskWaypoint:
    name: str
    joints: np.ndarray
    end_effector_xyz: np.ndarray


def _xyz(entry: dict[str, Any]) -> np.ndarray:
    return np.asarray(entry.get("xyz", [0.0, 0.0, 0.0])[:3], dtype=np.float64)


def _size(entry: dict[str, Any]) -> np.ndarray:
    size = entry.get("size", [0.0, 0.0, 0.0])
    return np.asarray((list(size) + [0.0, 0.0, 0.0])[:3], dtype=np.float64)


def forward_kinematics(joints: np.ndarray, robot_spawn_xyz=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Return the world-space centre of the SO-101 gripper jaws."""
    joints = np.asarray(joints, dtype=np.float64).reshape(6)
    spawn = np.asarray(robot_spawn_xyz, dtype=np.float64)
    cumulative_pitch = np.cumsum(joints[1:4])
    radial = float(np.sum(ARM_LINK_LENGTHS * np.sin(cumulative_pitch)))
    vertical = float(np.sum(ARM_LINK_LENGTHS * np.cos(cumulative_pitch)))
    return np.array([
        spawn[0] + radial * math.cos(float(joints[0])),
        spawn[1] + radial * math.sin(float(joints[0])),
        spawn[2] + SHOULDER_PIVOT_Z + vertical,
    ], dtype=np.float64)


def solve_position_ik(
    target_xyz,
    robot_spawn_xyz=(0.0, 0.0, 0.0),
    gripper_width=0.04,
    preferred_joints=None,
) -> np.ndarray:
    """Solve a position-only IK target with a downward-biased gripper pose."""
    target = np.asarray(target_xyz, dtype=np.float64)
    spawn = np.asarray(robot_spawn_xyz, dtype=np.float64)
    relative = target - spawn
    pan = math.atan2(float(relative[1]), float(relative[0]))
    if not JOINT_LIMITS[0, 0] <= pan <= JOINT_LIMITS[0, 1]:
        raise ValueError(f"SO-101 target requires shoulder pan {pan:.3f} outside limits")

    radial = math.hypot(float(relative[0]), float(relative[1]))
    vertical = float(relative[2] - SHOULDER_PIVOT_Z)
    l1, l2, l3 = ARM_LINK_LENGTHS
    preferred = None if preferred_joints is None else np.asarray(preferred_joints, dtype=np.float64)
    best = None

    # Scan tool pitch, then solve the remaining two-link arm analytically. The
    # scan makes the redundant 3-DOF pitch chain deterministic and avoids a
    # scipy dependency in the policy container.
    for tool_pitch in np.linspace(0.35, 3.8, 691):
        wrist_r = radial - l3 * math.sin(float(tool_pitch))
        wrist_z = vertical - l3 * math.cos(float(tool_pitch))
        cosine = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        if abs(cosine) > 1.0:
            continue
        elbow_magnitude = math.acos(max(-1.0, min(1.0, cosine)))
        for planar_elbow in (elbow_magnitude, -elbow_magnitude):
            planar_shoulder = math.atan2(wrist_z, wrist_r) - math.atan2(
                l2 * math.sin(planar_elbow), l1 + l2 * math.cos(planar_elbow)
            )
            shoulder = math.pi / 2.0 - planar_shoulder
            elbow = -planar_elbow
            wrist = float(tool_pitch) - shoulder - elbow
            pitch_joints = np.array([shoulder, elbow, wrist], dtype=np.float64)
            if np.any(pitch_joints < JOINT_LIMITS[1:4, 0]) or np.any(
                pitch_joints > JOINT_LIMITS[1:4, 1]
            ):
                continue
            candidate = np.array([pan, shoulder, elbow, wrist, 0.0, gripper_width])
            score = abs(float(tool_pitch) - 2.4) + 0.03 * float(np.dot(pitch_joints, pitch_joints))
            if preferred is not None:
                score += 0.12 * float(np.linalg.norm(candidate[:4] - preferred[:4]))
            if best is None or score < best[0]:
                best = (score, candidate)

    if best is None:
        raise ValueError(f"SO-101 target is unreachable: {target.tolist()}")
    result = np.clip(best[1], JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]).astype(np.float32)
    error = float(np.linalg.norm(forward_kinematics(result, spawn) - target))
    if error > 0.002:
        raise ValueError(f"SO-101 IK residual {error:.4f} m for target {target.tolist()}")
    return result


def _select_task(scenario: dict[str, Any]) -> dict[str, Any]:
    selected = scenario.get("selected_task")
    if selected:
        return selected
    tasks = scenario.get("tasks", [])
    if not tasks:
        raise ValueError("SO-101 demo scenario contains no task")
    return tasks[0]


def build_pick_place_plan(scenario: dict[str, Any]) -> list[TaskWaypoint]:
    """Build collision-conscious pick/place waypoints from a fixed scenario."""
    task = _select_task(scenario)
    objects = {entry.get("id"): entry for entry in scenario.get("objects", [])}
    assets = {entry.get("name"): entry for entry in scenario.get("static_assets", [])}
    try:
        target = objects[task["target_object"]]
        destination = assets[task["destination"]]
    except KeyError as exc:
        raise ValueError(f"SO-101 task references missing scenario entity: {exc}") from exc

    spawn = _xyz(scenario.get("robot_spawn", {}))
    target_xyz = _xyz(target)
    destination_xyz = _xyz(destination)
    place_xyz = destination_xyz.copy()
    place_xyz[2] += 0.5 * (_size(destination)[2] + _size(target)[2])
    transit_z = max(float(target_xyz[2] + 0.15), float(place_xyz[2] + 0.15))

    poses = [
        ("pregrasp", target_xyz + [0.0, 0.0, 0.12], 0.04),
        ("grasp", target_xyz, 0.04),
        ("close_gripper", target_xyz, 0.01),
        ("lift", np.array([target_xyz[0], target_xyz[1], transit_z]), 0.01),
        ("transfer", np.array([place_xyz[0], place_xyz[1], transit_z]), 0.01),
        ("place", place_xyz, 0.01),
        ("release", place_xyz, 0.04),
        ("retreat", place_xyz + [0.0, 0.0, 0.13], 0.04),
    ]

    plan = []
    preferred = None
    for name, xyz, gripper_width in poses:
        joints = solve_position_ik(xyz, spawn, gripper_width, preferred)
        plan.append(TaskWaypoint(name=name, joints=joints, end_effector_xyz=np.asarray(xyz)))
        preferred = joints
    return plan


class ScriptedPickPlaceController:
    """Advance through a task plan as measured joints reach each waypoint."""

    def __init__(self, tolerance=0.055, dwell_steps=2, max_step=0.08, scenario=None):
        # Isaac's force-driven joints settle up to 0.05 rad from a commanded
        # pose under gravity or while the gripper contacts the destination bin.
        # A tighter gate leaves an otherwise valid pick/place parked forever.
        self.tolerance = float(tolerance)
        self.dwell_steps = int(dwell_steps)
        self.max_step = float(max_step)
        self.scenario = scenario
        self.plan: list[TaskWaypoint] | None = None
        self.index = 0
        self.dwell = 0

    @property
    def status(self) -> str:
        if not self.plan:
            return "waiting_for_task"
        if self.index >= len(self.plan) - 1 and self.dwell >= self.dwell_steps:
            return "solved"
        return self.plan[self.index].name

    def reset(self):
        self.plan = None
        self.index = 0
        self.dwell = 0

    def get_action(self, observation: dict[str, Any]) -> np.ndarray:
        state = observation.get("state", {}).get("joint_positions")
        current = np.zeros(6, dtype=np.float32) if state is None else np.asarray(
            state, dtype=np.float32
        ).reshape(-1, 6)[-1]
        if self.plan is None:
            scenario = observation.get("metadata", {}).get("scenario") or self.scenario
            if not scenario:
                raise ValueError("SO-101 demo policy requires scenario metadata")
            self.plan = build_pick_place_plan(scenario)

        waypoint = self.plan[self.index]
        if float(np.max(np.abs(current - waypoint.joints))) <= self.tolerance:
            self.dwell += 1
            if self.dwell >= self.dwell_steps and self.index < len(self.plan) - 1:
                self.index += 1
                self.dwell = 0
                waypoint = self.plan[self.index]
        else:
            self.dwell = 0

        delta = np.clip(waypoint.joints - current, -self.max_step, self.max_step)
        return np.clip(current + delta, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]).astype(np.float32)

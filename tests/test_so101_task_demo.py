import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "applications" / "groot" / "scripts"))
sys.path.insert(0, str(ROOT / "applications" / "so101" / "scripts"))

from score_picking_task import score
from so101_common import JOINT_NAMES, safe_hardware_target
from so101_task_policy import (
    ScriptedPickPlaceController,
    build_pick_place_plan,
    forward_kinematics,
)


def load_scenario():
    return json.loads((ROOT / "systems" / "so101" / "scenarios" / "picking_table.json").read_text())


def test_task_plan_is_reachable_and_completes():
    scenario = load_scenario()
    for task in scenario["tasks"]:
        scenario["selected_task"] = task
        task_plan = build_pick_place_plan(scenario)
        assert len(task_plan) == 8
    scenario["selected_task"] = scenario["tasks"][0]
    plan = build_pick_place_plan(scenario)
    assert [waypoint.name for waypoint in plan] == [
        "pregrasp",
        "grasp",
        "close_gripper",
        "lift",
        "transfer",
        "place",
        "release",
        "retreat",
    ]
    spawn = scenario["robot_spawn"]["xyz"]
    for waypoint in plan:
        actual = forward_kinematics(waypoint.joints, spawn)
        np.testing.assert_allclose(actual, waypoint.end_effector_xyz, atol=2e-4)

    controller = ScriptedPickPlaceController()
    current = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    observation = {
        "state": {"joint_positions": current.reshape(1, 1, -1)},
        "metadata": {"scenario": scenario},
    }
    for _ in range(120):
        observation["state"]["joint_positions"] = current.reshape(1, 1, -1)
        current = controller.get_action(observation)
        if controller.status == "solved":
            break
    assert controller.status == "solved"


def test_scripted_task_telemetry_passes_acceptance_scorer():
    scenario = load_scenario()
    plan = build_pick_place_plan(scenario)
    target = next(entry for entry in scenario["objects"] if entry["id"] == "red_block_target")
    object_xyz = np.asarray(target["xyz"], dtype=np.float64)
    samples = []
    for index, waypoint in enumerate(plan):
        if waypoint.name in ("close_gripper", "lift", "transfer", "place", "release"):
            object_xyz = waypoint.end_effector_xyz.copy()
        samples.append({
            "time": float(index * 2),
            "end_effector_xyz": waypoint.end_effector_xyz.tolist(),
            "object_xyz": object_xyz.tolist(),
            "gripper_width_m": float(waypoint.joints[-1]),
            "policy_used_ground_truth": False,
        })
    telemetry = {
        "task_id": "pick_red_block_left_bin",
        "duration_s": samples[-1]["time"],
        "grasp_closed": True,
        "policy_used_ground_truth": False,
        "collisions": [],
        "failed_grasps": [],
        "samples": samples,
    }
    metrics = score(scenario, telemetry, telemetry["task_id"])
    assert metrics["success"] is True
    assert all(metrics["checks"].values())


def test_hil_safety_target_limits_step_and_joint_range():
    current = np.zeros(len(JOINT_NAMES), dtype=np.float64)
    requested = np.array([10.0, -10.0, 10.0, -10.0, 10.0, 1.0])
    result = safe_hardware_target(requested, current, 0.04)
    assert np.max(np.abs(result - current)) <= 0.04
    assert result[-1] == 0.04
    for invalid in (requested * np.nan, requested * np.inf):
        try:
            safe_hardware_target(invalid, current, 0.04)
        except ValueError:
            pass
        else:
            raise AssertionError("non-finite HIL command was accepted")

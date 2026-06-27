#!/usr/bin/env python3
"""Score SO-101 tabletop picking task telemetry against scenario success rules."""

import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_THRESHOLDS = {
    "approach_tolerance_m": 0.035,
    "grasp_close_width_m": 0.018,
    "workspace_radius_m": 0.62,
    "workspace_min_z_m": 0.40,
    "recovery_window_s": 20.0,
}


def load_json(path):
    return json.loads(Path(path).read_text())


def norm3(values):
    arr = np.asarray(values[:3], dtype=np.float64)
    return float(np.linalg.norm(arr))


def xyz(entry, key, default=None):
    value = entry.get(key, default)
    if value is None:
        return None
    if isinstance(value, dict):
        value = value.get("xyz")
    if value is None:
        return None
    return np.asarray(value[:3], dtype=np.float64)


def select_task(scenario, task_id):
    tasks = scenario.get("tasks", [])
    if task_id:
        for task in tasks:
            if task.get("id") == task_id:
                return task
        raise SystemExit(f"Task {task_id!r} not found in scenario")
    if not tasks:
        raise SystemExit("Scenario has no tasks")
    return tasks[0]


def object_by_id(scenario, object_id):
    for obj in scenario.get("objects", []):
        if obj.get("id") == object_id:
            return obj
    raise SystemExit(f"Object {object_id!r} not found in scenario")


def asset_by_name(scenario, name):
    for asset in scenario.get("static_assets", []):
        if asset.get("name") == name:
            return asset
    raise SystemExit(f"Destination {name!r} not found in scenario")


def min_distance(samples, a_key, b_key):
    distances = []
    for sample in samples:
        a = xyz(sample, a_key)
        b = xyz(sample, b_key)
        if a is not None and b is not None:
            distances.append(float(np.linalg.norm(a - b)))
    return min(distances) if distances else math.inf


def max_lift(samples, object_key, initial_z):
    heights = []
    for sample in samples:
        obj = xyz(sample, object_key)
        if obj is not None:
            heights.append(float(obj[2] - initial_z))
    return max(heights) if heights else -math.inf


def final_place_error(samples, object_key, destination_xyz):
    for sample in reversed(samples):
        obj = xyz(sample, object_key)
        if obj is not None:
            return float(np.linalg.norm(obj[:2] - destination_xyz[:2]))
    return math.inf


def has_ground_truth_leak(telemetry):
    if telemetry.get("policy_used_ground_truth", False):
        return True
    for sample in telemetry.get("samples", []):
        if sample.get("policy_used_ground_truth", False):
            return True
    return False


def recovery_success(telemetry, success, window_s):
    failures = telemetry.get("failed_grasps", [])
    if not failures:
        return True
    if not success:
        return False
    finish = float(telemetry.get("duration_s", telemetry.get("samples", [{}])[-1].get("time", math.inf)))
    return any(finish - float(failure.get("time", -math.inf)) <= window_s for failure in failures)


def score(scenario, telemetry, task_id=""):
    task = select_task(scenario, task_id or telemetry.get("task_id", ""))
    target = object_by_id(scenario, task["target_object"])
    destination = asset_by_name(scenario, task["destination"])
    success_rules = task.get("success", {})
    thresholds = {**DEFAULT_THRESHOLDS, **telemetry.get("thresholds", {})}
    samples = telemetry.get("samples", [])

    target_initial = np.asarray(target["xyz"][:3], dtype=np.float64)
    destination_xyz = np.asarray(destination["xyz"][:3], dtype=np.float64)
    min_approach = min_distance(samples, "end_effector_xyz", "object_xyz")
    lift_height = max_lift(samples, "object_xyz", target_initial[2])
    place_error = final_place_error(samples, "object_xyz", destination_xyz)
    collision_count = len(telemetry.get("collisions", []))
    max_collisions = int(success_rules.get("max_collision_count", 0))
    duration = float(telemetry.get("duration_s", samples[-1].get("time", math.inf) if samples else math.inf))

    reachability = norm3(target_initial) <= thresholds["workspace_radius_m"] and target_initial[2] >= thresholds["workspace_min_z_m"]
    approach_ok = min_approach <= thresholds["approach_tolerance_m"]
    grasp_closed = bool(telemetry.get("grasp_closed", False)) or any(
        float(sample.get("gripper_width_m", math.inf)) <= thresholds["grasp_close_width_m"]
        for sample in samples
    )
    lift_ok = lift_height >= float(success_rules.get("object_lift_height_m", 0.0))
    place_ok = place_error <= float(success_rules.get("place_position_tolerance_m", 0.05))
    collision_free = collision_count <= max_collisions
    duration_ok = duration <= float(success_rules.get("max_duration_s", math.inf))
    no_ground_truth = not has_ground_truth_leak(telemetry)
    recovered = recovery_success(telemetry, place_ok, thresholds["recovery_window_s"])

    checks = {
        "reachable_target": bool(reachability),
        "grasp_approach": bool(approach_ok),
        "grasp_closure": bool(grasp_closed),
        "lift_stability": bool(lift_ok),
        "place_accuracy": bool(place_ok),
        "collision_free": bool(collision_free),
        "duration": bool(duration_ok),
        "failed_grasp_recovery": bool(recovered),
        "no_policy_ground_truth": bool(no_ground_truth),
    }
    metrics = {
        "task_id": task["id"],
        "target_object": task["target_object"],
        "destination": task["destination"],
        "min_approach_distance_m": min_approach,
        "max_lift_height_m": lift_height,
        "final_place_error_m": place_error,
        "collision_count": collision_count,
        "duration_s": duration,
        "success": all(checks.values()),
        "checks": checks,
    }
    return metrics


def randomized_scenarios(scenario, count, seed):
    rng = np.random.default_rng(seed)
    limits = scenario.get("domain_randomization", {})
    xy_jitter = float(limits.get("object_xy_jitter_m", 0.0))
    yaw_jitter = float(limits.get("object_yaw_jitter_rad", 0.0))
    mass_scale = limits.get("object_mass_scale", [1.0, 1.0])
    for index in range(count):
        variant = copy.deepcopy(scenario)
        variant["name"] = f"{scenario.get('name', 'so101_scenario')}_seed_{seed + index}"
        variant["randomization_seed"] = seed + index
        for obj in variant.get("objects", []):
            obj["xyz"][0] += float(rng.uniform(-xy_jitter, xy_jitter))
            obj["xyz"][1] += float(rng.uniform(-xy_jitter, xy_jitter))
            obj.setdefault("rpy", [0.0, 0.0, 0.0])[2] += float(rng.uniform(-yaw_jitter, yaw_jitter))
            obj["mass_kg"] = float(obj.get("mass_kg", 0.04) * rng.uniform(mass_scale[0], mass_scale[1]))
        yield variant


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--telemetry", help="JSON telemetry file to score")
    parser.add_argument("--task-id", default="")
    parser.add_argument("--output", help="Write metrics JSON to this path")
    parser.add_argument("--write-randomized-scenarios", help="Directory for randomized scenario variants")
    parser.add_argument("--num-randomizations", type=int, default=0)
    parser.add_argument("--seed", type=int, default=101)
    args = parser.parse_args()

    scenario = load_json(args.scenario)
    if args.write_randomized_scenarios:
        output_dir = Path(args.write_randomized_scenarios)
        output_dir.mkdir(parents=True, exist_ok=True)
        for variant in randomized_scenarios(scenario, args.num_randomizations, args.seed):
            (output_dir / f"{variant['name']}.json").write_text(json.dumps(variant, indent=2) + "\n")

    if not args.telemetry:
        return

    metrics = score(scenario, load_json(args.telemetry), args.task_id)
    rendered = json.dumps(metrics, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end="")
    if not metrics["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

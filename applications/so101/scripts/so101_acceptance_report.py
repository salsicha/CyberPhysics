#!/usr/bin/env python3
"""Aggregate SO-101 picking validation metrics into acceptance checks."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


DEFAULT_THRESHOLDS = {
    "min_success_rate": 0.90,
    "max_mean_place_error_m": 0.035,
    "max_worst_place_error_m": 0.055,
    "max_collision_count_total": 0,
    "max_joint_limit_violations_total": 0,
    "max_command_saturation_fraction": 0.05,
    "min_failed_grasp_recovery_rate": 0.80,
    "max_policy_latency_ms": 120.0,
    "max_observation_age_ms": 150.0,
}


def load_json(path):
    return json.loads(Path(path).read_text())


def load_runs(paths):
    runs = []
    for path in paths:
        path = Path(path)
        if path.is_dir():
            for child in sorted(path.glob("*.json")):
                runs.append(load_json(child))
        else:
            runs.append(load_json(path))
    if not runs:
        raise SystemExit("No run metric files provided")
    return runs


def run_success(run):
    if "success" in run:
        return bool(run["success"])
    return all(run.get("checks", {}).values())


def run_group(run):
    object_class = run.get("object_class") or run.get("target_object", "unknown")
    clutter = run.get("clutter_level", "default")
    return object_class, clutter


def acceptance_report(runs, thresholds):
    success = np.asarray([run_success(run) for run in runs], dtype=bool)
    place_errors = np.asarray([float(run.get("final_place_error_m", np.inf)) for run in runs], dtype=np.float64)
    collisions = int(sum(int(run.get("collision_count", 0)) for run in runs))
    joint_violations = int(sum(int(run.get("joint_limit_violations", 0)) for run in runs))
    saturation = [float(run.get("command_saturation_fraction", 0.0)) for run in runs]
    recoveries = []
    latencies = []
    observation_ages = []
    grouped = defaultdict(list)

    for run in runs:
        grouped[run_group(run)].append(run_success(run))
        checks = run.get("checks", {})
        if "failed_grasp_recovery" in checks:
            recoveries.append(bool(checks["failed_grasp_recovery"]))
        elif "failed_grasp_recovery" in run:
            recoveries.append(bool(run["failed_grasp_recovery"]))
        if "policy_latency_ms" in run:
            latencies.append(float(run["policy_latency_ms"]))
        if "observation_age_ms" in run:
            observation_ages.append(float(run["observation_age_ms"]))

    success_rate = float(np.mean(success))
    mean_place_error = float(np.mean(place_errors))
    worst_place_error = float(np.max(place_errors))
    max_saturation = float(np.max(saturation)) if saturation else 0.0
    recovery_rate = float(np.mean(recoveries)) if recoveries else 1.0
    max_latency = float(np.max(latencies)) if latencies else 0.0
    max_observation_age = float(np.max(observation_ages)) if observation_ages else 0.0

    group_rates = {
        f"{object_class}:{clutter}": float(np.mean(values))
        for (object_class, clutter), values in sorted(grouped.items())
    }
    checks = {
        "success_rate": success_rate >= thresholds["min_success_rate"],
        "success_rate_by_object_and_clutter": all(
            rate >= thresholds["min_success_rate"] for rate in group_rates.values()
        ),
        "mean_place_error": mean_place_error <= thresholds["max_mean_place_error_m"],
        "worst_place_error": worst_place_error <= thresholds["max_worst_place_error_m"],
        "collision_count": collisions <= thresholds["max_collision_count_total"],
        "joint_limit_violations": joint_violations <= thresholds["max_joint_limit_violations_total"],
        "command_saturation": max_saturation <= thresholds["max_command_saturation_fraction"],
        "failed_grasp_recovery": recovery_rate >= thresholds["min_failed_grasp_recovery_rate"],
        "policy_latency": max_latency <= thresholds["max_policy_latency_ms"],
        "observation_age": max_observation_age <= thresholds["max_observation_age_ms"],
    }
    return {
        "run_count": len(runs),
        "success": all(checks.values()),
        "checks": {name: bool(value) for name, value in checks.items()},
        "metrics": {
            "success_rate": success_rate,
            "success_rate_by_object_and_clutter": group_rates,
            "mean_place_error_m": mean_place_error,
            "worst_place_error_m": worst_place_error,
            "collision_count_total": collisions,
            "joint_limit_violations_total": joint_violations,
            "max_command_saturation_fraction": max_saturation,
            "failed_grasp_recovery_rate": recovery_rate,
            "max_policy_latency_ms": max_latency,
            "max_observation_age_ms": max_observation_age,
        },
        "thresholds": thresholds,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics", nargs="+", required=True, help="Metric JSON files or directories")
    parser.add_argument("--thresholds", default="systems/so101/validation/acceptance_thresholds.json")
    parser.add_argument("--output")
    args = parser.parse_args()

    thresholds = {**DEFAULT_THRESHOLDS}
    threshold_path = Path(args.thresholds)
    if threshold_path.exists():
        thresholds.update(load_json(threshold_path))
    report = acceptance_report(load_runs(args.metrics), thresholds)
    rendered = json.dumps(report, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end="")
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

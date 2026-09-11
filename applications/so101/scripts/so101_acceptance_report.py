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
        return run["success"] is True
    checks = run.get("checks")
    if not isinstance(checks, dict) or not checks:
        return False
    return all(value is True for value in checks.values())


def run_group(run):
    object_class = run.get("object_class") or run.get("target_object", "unknown")
    clutter = run.get("clutter_level", "default")
    return object_class, clutter


def acceptance_report(runs, thresholds):
    if not runs:
        raise ValueError("At least one run is required for acceptance")
    measurement_errors = []

    def measured(field, *, integer=False, maximum=None):
        values = []
        for index, run in enumerate(runs):
            value = run.get(field)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not np.isfinite(value) or value < 0
                    or (integer and value != int(value))
                    or (maximum is not None and value > maximum)):
                measurement_errors.append(f"run {index}: missing or invalid {field}")
                values.append(np.nan)
            else:
                values.append(float(value))
        return np.asarray(values)

    def aggregate(values, reducer):
        if not np.all(np.isfinite(values)):
            return None
        result = float(reducer(values))
        return result if np.isfinite(result) else None

    success = np.asarray([run_success(run) for run in runs], dtype=bool)
    place_errors = measured("final_place_error_m")
    collisions = aggregate(measured("collision_count", integer=True), np.sum)
    joint_violations = aggregate(measured("joint_limit_violations", integer=True), np.sum)
    saturation = measured("command_saturation_fraction", maximum=1)
    recoveries = []
    latencies = measured("policy_latency_ms")
    observation_ages = measured("observation_age_ms")
    grouped = defaultdict(list)

    for index, run in enumerate(runs):
        grouped[run_group(run)].append(run_success(run))
        checks = run.get("checks")
        if not isinstance(checks, dict):
            checks = {}
        recovery = checks.get("failed_grasp_recovery", run.get("failed_grasp_recovery"))
        if not isinstance(recovery, bool):
            measurement_errors.append(f"run {index}: missing or invalid failed_grasp_recovery")
            recoveries.append(np.nan)
        else:
            recoveries.append(float(recovery))

    success_rate = float(np.mean(success))
    mean_place_error = aggregate(place_errors, np.mean)
    worst_place_error = aggregate(place_errors, np.max)
    max_saturation = aggregate(saturation, np.max)
    recovery_rate = aggregate(recoveries, np.mean)
    max_latency = aggregate(latencies, np.max)
    max_observation_age = aggregate(observation_ages, np.max)

    group_rates = {
        f"{object_class}:{clutter}": float(np.mean(values))
        for (object_class, clutter), values in sorted(grouped.items())
    }
    checks = {
        "success_rate": success_rate >= thresholds["min_success_rate"],
        "success_rate_by_object_and_clutter": all(
            rate >= thresholds["min_success_rate"] for rate in group_rates.values()
        ),
        "mean_place_error": mean_place_error is not None and mean_place_error <= thresholds["max_mean_place_error_m"],
        "worst_place_error": worst_place_error is not None and worst_place_error <= thresholds["max_worst_place_error_m"],
        "collision_count": collisions is not None and collisions <= thresholds["max_collision_count_total"],
        "joint_limit_violations": joint_violations is not None and joint_violations <= thresholds["max_joint_limit_violations_total"],
        "command_saturation": max_saturation is not None and max_saturation <= thresholds["max_command_saturation_fraction"],
        "failed_grasp_recovery": recovery_rate is not None and recovery_rate >= thresholds["min_failed_grasp_recovery_rate"],
        "policy_latency": max_latency is not None and max_latency <= thresholds["max_policy_latency_ms"],
        "observation_age": max_observation_age is not None and max_observation_age <= thresholds["max_observation_age_ms"],
    }
    return {
        "run_count": len(runs),
        "success": all(checks.values()),
        "checks": {name: bool(value) for name, value in checks.items()},
        "measurement_errors": measurement_errors,
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

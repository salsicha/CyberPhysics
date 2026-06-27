#!/usr/bin/env python3
"""Aggregate RACECAR Neo mission metrics into acceptance checks."""

import argparse
import json
from pathlib import Path

import numpy as np


DEFAULT_THRESHOLDS = {
    'min_waypoint_completion_rate': 0.95,
    'max_nominal_collision_count': 0,
    'max_recovery_collision_speed_mps': 0.08,
    'max_lateral_tracking_error_m': 0.45,
    'max_stop_line_error_m': 0.35,
    'max_localization_drift_m': 0.45,
    'max_costmap_age_s': 1.0,
    'max_command_saturation_fraction': 0.05,
    'max_actuator_lag_s': 0.25,
}


def load_json(path):
    return json.loads(Path(path).read_text())


def load_runs(paths):
    runs = []
    for item in paths:
        path = Path(item)
        if path.is_dir():
            runs.extend(load_json(child) for child in sorted(path.glob('*.json')))
        else:
            runs.append(load_json(path))
    if not runs:
        raise SystemExit('No Racecar Neo metric files provided')
    return runs


def value(run, key, default=0.0):
    return float(run.get(key, default))


def completion_rate(run):
    if 'waypoint_completion_rate' in run:
        return value(run, 'waypoint_completion_rate')
    reached = float(run.get('waypoints_reached', 0))
    total = float(run.get('waypoints_total', 0))
    return reached / total if total > 0 else 0.0


def acceptance_report(runs, thresholds):
    completion = np.asarray([completion_rate(run) for run in runs], dtype=np.float64)
    nominal_collisions = sum(int(run.get('collision_count', 0)) for run in runs if run.get('run_type', 'nominal') == 'nominal')
    recovery_collision_speeds = [value(run, 'max_collision_speed_mps') for run in runs if run.get('run_type') == 'recovery']
    max_recovery_collision_speed = max(recovery_collision_speeds) if recovery_collision_speeds else 0.0
    max_lateral = max(value(run, 'max_lateral_error_m') for run in runs)
    max_stop_line = max(value(run, 'max_stop_line_error_m') for run in runs)
    max_drift = max(value(run, 'max_localization_drift_m') for run in runs)
    max_costmap_age = max(value(run, 'max_costmap_age_s') for run in runs)
    max_saturation = max(value(run, 'command_saturation_fraction') for run in runs)
    max_lag = max(value(run, 'max_actuator_lag_s') for run in runs)

    checks = {
        'waypoint_completion_rate': float(np.mean(completion)) >= thresholds['min_waypoint_completion_rate'],
        'nominal_collision_count': nominal_collisions <= int(thresholds['max_nominal_collision_count']),
        'recovery_collision_speed': max_recovery_collision_speed <= thresholds['max_recovery_collision_speed_mps'],
        'lateral_tracking_error': max_lateral <= thresholds['max_lateral_tracking_error_m'],
        'stop_line_error': max_stop_line <= thresholds['max_stop_line_error_m'],
        'localization_drift': max_drift <= thresholds['max_localization_drift_m'],
        'costmap_freshness': max_costmap_age <= thresholds['max_costmap_age_s'],
        'command_saturation': max_saturation <= thresholds['max_command_saturation_fraction'],
        'actuator_lag': max_lag <= thresholds['max_actuator_lag_s'],
    }
    return {
        'run_count': len(runs),
        'success': all(checks.values()),
        'checks': {name: bool(result) for name, result in checks.items()},
        'metrics': {
            'mean_waypoint_completion_rate': float(np.mean(completion)),
            'worst_waypoint_completion_rate': float(np.min(completion)),
            'nominal_collision_count': nominal_collisions,
            'max_recovery_collision_speed_mps': max_recovery_collision_speed,
            'max_lateral_tracking_error_m': max_lateral,
            'max_stop_line_error_m': max_stop_line,
            'max_localization_drift_m': max_drift,
            'max_costmap_age_s': max_costmap_age,
            'max_command_saturation_fraction': max_saturation,
            'max_actuator_lag_s': max_lag,
        },
        'thresholds': thresholds,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metrics', nargs='+', required=True, help='Metric JSON files or directories')
    parser.add_argument('--thresholds', default='systems/racecarneo/validation/acceptance_thresholds.json')
    parser.add_argument('--output')
    args = parser.parse_args()

    thresholds = {**DEFAULT_THRESHOLDS}
    threshold_path = Path(args.thresholds)
    if threshold_path.exists():
        thresholds.update(load_json(threshold_path))
    report = acceptance_report(load_runs(args.metrics), thresholds)
    rendered = json.dumps(report, indent=2) + '\n'
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end='')
    if not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

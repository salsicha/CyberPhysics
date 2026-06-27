#!/usr/bin/env python3
"""Score RACECAR Neo sensor-driven behavior telemetry."""

import argparse
import json
import math
from pathlib import Path


def load_json(path):
    return json.loads(Path(path).read_text())


def select_test(manifest, test_id):
    tests = manifest.get('tests', [])
    if test_id:
        for test in tests:
            if test.get('id') == test_id:
                return test
        raise SystemExit(f'Test {test_id!r} not found')
    if not tests:
        raise SystemExit('No tests in behavior manifest')
    return tests[0]


def max_sample(samples, key, default=0.0):
    values = [float(sample[key]) for sample in samples if key in sample]
    return max(values) if values else default


def min_sample(samples, key, default=math.inf):
    values = [float(sample[key]) for sample in samples if key in sample]
    return min(values) if values else default


def stale_depth_unsafe_speed(samples, stale_threshold_s, speed_limit_mps):
    unsafe = []
    for sample in samples:
        depth_age = float(sample.get('depth_age_s', 0.0))
        speed = abs(float(sample.get('speed_mps', 0.0)))
        if depth_age > stale_threshold_s and speed > speed_limit_mps:
            unsafe.append({"time": sample.get('time'), "depth_age_s": depth_age, "speed_mps": speed})
    return unsafe


def score(manifest, telemetry, test_id=''):
    test = select_test(manifest, test_id or telemetry.get('test_id', ''))
    rules = test.get('success', {})
    samples = telemetry.get('samples', [])
    forbidden = set(manifest.get('ground_truth_policy', {}).get('forbidden_runtime_topics', []))
    runtime_inputs = set(telemetry.get('planner_inputs', []))
    forbidden_used = sorted(forbidden & runtime_inputs)
    if telemetry.get('ground_truth_used_by_planner', False):
        forbidden_used.append('ground_truth_used_by_planner')

    stale_depth_limit = float(rules.get('max_stale_depth_speed_mps', telemetry.get('max_stale_depth_speed_mps', 0.08)))
    stale_depth_threshold = float(telemetry.get('stale_depth_threshold_s', 1.0))
    stale_depth_violations = stale_depth_unsafe_speed(samples, stale_depth_threshold, stale_depth_limit)

    collision_count = int(telemetry.get('collision_count', 0))
    recovery_count = int(telemetry.get('recovery_count', 0))
    recovery_time = float(telemetry.get('recovery_time_s', 0.0))
    metrics = {
        'test_id': test['id'],
        'mission_id': test.get('mission_id'),
        'collision_count': collision_count,
        'max_lateral_error_m': max_sample(samples, 'lateral_error_m'),
        'max_localization_drift_m': max_sample(samples, 'localization_drift_m'),
        'max_costmap_age_s': max_sample(samples, 'costmap_age_s'),
        'min_obstacle_clearance_m': min_sample(samples, 'obstacle_clearance_m'),
        'recovery_count': recovery_count,
        'recovery_time_s': recovery_time,
        'stale_depth_violations': stale_depth_violations,
        'forbidden_ground_truth_inputs': forbidden_used
    }
    checks = {
        'no_runtime_ground_truth': len(forbidden_used) == 0,
        'localized': (not rules.get('requires_localization', False)) or bool(telemetry.get('localized', False)),
        'map_consumed': (not rules.get('requires_map_consumption', False)) or bool(telemetry.get('map_consumed', False)),
        'collision_count': collision_count <= int(rules.get('max_collision_count', collision_count)),
        'lateral_error': metrics['max_lateral_error_m'] <= float(rules.get('max_lateral_error_m', math.inf)),
        'localization_drift': metrics['max_localization_drift_m'] <= float(rules.get('max_localization_drift_m', math.inf)),
        'costmap_age': metrics['max_costmap_age_s'] <= float(rules.get('max_costmap_age_s', math.inf)),
        'obstacle_clearance': metrics['min_obstacle_clearance_m'] >= float(rules.get('min_obstacle_clearance_m', -math.inf)),
        'recovery_completed': (not rules.get('requires_recovery', False)) or bool(telemetry.get('recovery_completed', False)),
        'recovery_count': recovery_count <= int(rules.get('max_recovery_count', recovery_count)),
        'recovery_time': recovery_time <= float(rules.get('max_recovery_time_s', math.inf)),
        'stale_depth_throttle': len(stale_depth_violations) == 0
    }
    return {
        'success': all(checks.values()),
        'checks': {name: bool(value) for name, value in checks.items()},
        'metrics': metrics,
        'degradations': test.get('degradations', [])
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', default='systems/racecarneo/validation/sensor_behavior_tests.json')
    parser.add_argument('--telemetry', required=True)
    parser.add_argument('--test-id', default='')
    parser.add_argument('--output')
    args = parser.parse_args()

    report = score(load_json(args.manifest), load_json(args.telemetry), args.test_id)
    rendered = json.dumps(report, indent=2) + '\n'
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end='')
    if not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Score turret tracking simulation telemetry against acceptance thresholds."""

import argparse
import json
from pathlib import Path


DEFAULT_THRESHOLDS = {
    'min_lock_fraction': 0.82,
    'max_mean_pixel_error_px': 34.0,
    'max_p95_pixel_error_px': 70.0,
    'max_mean_angular_error_deg': 3.2,
    'max_p95_angular_error_deg': 6.5,
    'min_target_detection_fraction': 0.88,
    'min_target_mean_iou': 0.68,
    'max_false_lock_count': 0,
    'max_reacquire_time_s': 1.2,
    'max_command_saturation_fraction': 0.08,
    'max_joint_limit_violation_count': 0,
    'max_emergency_stop_violation_count': 0,
    'max_yolo_latency_ms': 35.0,
}

REQUIRED_TOPICS = [
    '/turret/camera/image_raw',
    '/turret/camera/camera_info',
    '/turret/yolo/segments',
    '/turret/yolo/target_mask',
    '/turret/yolo/target_bbox',
    '/turret/yolo/target_confidence',
    '/turret/target/pixel_error',
    '/turret/target/angular_error',
    '/turret/target/selected_id',
    '/turret/target/locked',
    '/turret/joint_commands',
    '/turret/joint_states',
    '/turret/diagnostics',
    '/turret/emergency_stop',
]


def load_json(path):
    return json.loads(Path(path).read_text())


def load_runs(paths):
    runs = []
    for raw_path in paths:
        path = Path(raw_path)
        if path.is_dir():
            for child in sorted(path.glob('*.json')):
                runs.append(load_json(child))
        else:
            runs.append(load_json(path))
    if not runs:
        raise SystemExit('No turret metric files provided')
    return runs


def topic_contract_errors(run):
    samples = run.get('samples', [])
    if not samples:
        return ['missing samples']
    errors = []
    for index, sample in enumerate(samples[:10]):
        topics = sample.get('topics', {})
        missing = [topic for topic in REQUIRED_TOPICS if topic not in topics]
        if missing:
            errors.append(f'sample {index}: missing topics {missing}')
        joint_cmd = topics.get('/turret/joint_commands', {})
        joint_state = topics.get('/turret/joint_states', {})
        if joint_cmd.get('name') != ['pan_joint', 'tilt_joint']:
            errors.append(f'sample {index}: invalid joint command names')
        if joint_state.get('name') != ['pan_joint', 'tilt_joint']:
            errors.append(f'sample {index}: invalid joint state names')
        camera = topics.get('/turret/camera/camera_info', {})
        k = camera.get('k', [])
        if len(k) != 9 or not all(isinstance(value, (int, float)) for value in k):
            errors.append(f'sample {index}: invalid camera intrinsics')
    return errors


def score_one(run, thresholds):
    summary = run.get('summary', {})
    errors = topic_contract_errors(run)
    checks = {
        'topic_contract': not errors,
        'lock_fraction': summary.get('lock_fraction', 0.0) >= thresholds['min_lock_fraction'],
        'mean_pixel_error': summary.get('mean_pixel_error_px', float('inf')) <= thresholds['max_mean_pixel_error_px'],
        'p95_pixel_error': summary.get('p95_pixel_error_px', float('inf')) <= thresholds['max_p95_pixel_error_px'],
        'mean_angular_error': summary.get('mean_angular_error_deg', float('inf')) <= thresholds['max_mean_angular_error_deg'],
        'p95_angular_error': summary.get('p95_angular_error_deg', float('inf')) <= thresholds['max_p95_angular_error_deg'],
        'target_detection_fraction': summary.get('target_detection_fraction', 0.0) >= thresholds['min_target_detection_fraction'],
        'target_mean_iou': summary.get('target_mean_iou', 0.0) >= thresholds['min_target_mean_iou'],
        'false_lock_count': summary.get('false_lock_count', 1) <= thresholds['max_false_lock_count'],
        'reacquire_time': summary.get('max_reacquire_time_s', float('inf')) <= thresholds['max_reacquire_time_s'],
        'command_saturation': summary.get('command_saturation_fraction', 1.0) <= thresholds['max_command_saturation_fraction'],
        'joint_limit_violations': summary.get('joint_limit_violation_count', 1) <= thresholds['max_joint_limit_violation_count'],
        'emergency_stop_violations': summary.get('emergency_stop_violation_count', 1) <= thresholds['max_emergency_stop_violation_count'],
        'yolo_latency': summary.get('max_yolo_latency_ms', float('inf')) <= thresholds['max_yolo_latency_ms'],
    }
    return {
        'scenario': run.get('scenario', 'unknown'),
        'target_id': run.get('target_id', 'unknown'),
        'success': all(checks.values()),
        'checks': {name: bool(value) for name, value in checks.items()},
        'topic_contract_errors': errors,
        'metrics': summary,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metrics', nargs='+', required=True, help='Metric JSON files or directories')
    parser.add_argument('--thresholds', default='systems/turret/validation/acceptance_thresholds.json')
    parser.add_argument('--output')
    args = parser.parse_args()

    thresholds = {**DEFAULT_THRESHOLDS}
    threshold_path = Path(args.thresholds)
    if threshold_path.exists():
        thresholds.update(load_json(threshold_path))
    results = [score_one(run, thresholds) for run in load_runs(args.metrics)]
    report = {
        'run_count': len(results),
        'success': all(result['success'] for result in results),
        'results': results,
        'thresholds': thresholds,
    }
    rendered = json.dumps(report, indent=2) + '\n'
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end='')
    if not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

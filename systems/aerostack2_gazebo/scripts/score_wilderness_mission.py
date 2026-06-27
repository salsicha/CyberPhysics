#!/usr/bin/env python3
"""Score Aerodrone wilderness mission telemetry against the mission contract."""

import argparse
import json
import math
from pathlib import Path


def _distance_xy(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _position(sample):
    if 'position_enu_m' in sample:
        return sample['position_enu_m']
    return [
        sample['x_m'],
        sample['y_m'],
        sample.get('z_m', sample.get('agl_m', 0.0)),
    ]


def _point_in_polygon(point, polygon):
    x, y = point[:2]
    inside = False
    j = len(polygon) - 1
    for i, vertex in enumerate(polygon):
        xi, yi = vertex
        xj, yj = polygon[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_intersect = (xj - xi) * (y - yi) / max(1e-9, yj - yi) + xi
            if x < x_intersect:
                inside = not inside
        j = i
    return inside


def _waypoints(mission):
    points = []
    for phase in mission.get('phases', []):
        for waypoint in phase.get('waypoints', []):
            points.append(waypoint)
    return points


def _load_mission(path, mission_id):
    mission_file = json.loads(Path(path).read_text())
    missions = mission_file.get('missions', [])
    if mission_id:
        missions = [mission for mission in missions if mission['id'] == mission_id]
        if not missions:
            raise SystemExit(f'unknown mission id: {mission_id}')
    return mission_file, missions


def _validate_contract(mission_file, missions):
    errors = []
    geofence = mission_file.get('geofence', {})
    if not geofence.get('keep_in_polygon_enu_m'):
        errors.append('missing keep-in geofence')
    if not missions:
        errors.append('missing missions')
    for mission in missions:
        if not _waypoints(mission):
            errors.append(f'{mission["id"]}: missing waypoints')
        if 'score' not in mission:
            errors.append(f'{mission["id"]}: missing score contract')
        for waypoint in _waypoints(mission):
            if 'position_enu_m' not in waypoint:
                errors.append(
                    f'{mission["id"]}/{waypoint.get("id", "?")}: '
                    'missing position_enu_m')
            if 'target_agl_m' not in waypoint:
                errors.append(
                    f'{mission["id"]}/{waypoint.get("id", "?")}: '
                    'missing target_agl_m')
    return errors


def _score_mission(mission_file, mission, samples):
    waypoints = _waypoints(mission)
    completed = []
    max_wp_error = 0.0
    for waypoint in waypoints:
        target = waypoint['position_enu_m']
        radius = float(waypoint.get('acceptance_radius_m', 8.0))
        best = min(
            (_distance_xy(_position(sample), target) for sample in samples),
            default=float('inf'))
        completed.append(best <= radius)
        if math.isfinite(best):
            max_wp_error = max(max_wp_error, best)

    geofence = mission_file['geofence']
    keep_in = geofence['keep_in_polygon_enu_m']
    no_fly = geofence.get('no_fly_polygons_enu_m', [])
    min_agl = float(mission.get('score', {}).get(
        'min_terrain_clearance_m', geofence['min_agl_m']))
    max_agl = float(mission.get('score', {}).get(
        'max_agl_m', geofence['max_agl_m']))
    geofence_violations = 0
    no_fly_intrusions = 0
    min_seen_agl = float('inf')
    max_seen_agl = 0.0
    correction_samples = 0
    confident_corrections = 0
    min_confidence = float(
        mission_file['global_acceptance']['min_correction_confidence'])
    landed = False
    landing_error_m = None

    for sample in samples:
        pos = _position(sample)
        agl = float(sample.get('agl_m', pos[2]))
        min_seen_agl = min(min_seen_agl, agl)
        max_seen_agl = max(max_seen_agl, agl)
        if sample.get('geofence_violation') or not _point_in_polygon(pos, keep_in):
            geofence_violations += 1
        if any(_point_in_polygon(pos, polygon) for polygon in no_fly):
            no_fly_intrusions += 1
        source = str(sample.get('correction_source', sample.get('source', '')))
        confidence = float(sample.get('correction_confidence', 0.0))
        if (
                source in {'demnav', 'wildnav', 'fused'}
                or source.startswith('demnav')
                or source.startswith('wildnav')):
            correction_samples += 1
            if confidence >= min_confidence:
                confident_corrections += 1
        landed = landed or bool(sample.get('landed', False))

    if mission.get('landing_zones') and samples:
        final_pos = _position(samples[-1])
        landing_error_m = min(
            _distance_xy(final_pos, zone['center_enu_m'])
            for zone in mission['landing_zones'])

    completion = sum(completed) / max(1, len(completed))
    correction_coverage = confident_corrections / max(1, correction_samples)
    score = mission.get('score', {})
    pass_fail = {
        'waypoint_completion': completion >= float(
            score.get('min_waypoint_completion', 1.0)),
        'geofence': geofence_violations <= mission_file[
            'global_acceptance']['max_geofence_violations'],
        'no_fly': no_fly_intrusions <= int(score.get('max_no_fly_intrusions', 0)),
        'terrain_clearance': min_seen_agl >= min_agl and max_seen_agl <= max_agl,
    }
    if score.get('require_landed'):
        pass_fail['landed'] = landed
    if landing_error_m is not None:
        pass_fail['landing_error'] = landing_error_m <= float(
            score.get('max_touchdown_error_m', 5.0))

    return {
        'mission_id': mission['id'],
        'samples': len(samples),
        'waypoint_completion': completion,
        'completed_waypoints': [
            wp['id'] for wp, done in zip(waypoints, completed) if done],
        'missed_waypoints': [
            wp['id'] for wp, done in zip(waypoints, completed) if not done],
        'max_waypoint_error_m': max_wp_error,
        'min_agl_m': None if min_seen_agl == float('inf') else min_seen_agl,
        'max_agl_m': max_seen_agl,
        'geofence_violations': geofence_violations,
        'no_fly_intrusions': no_fly_intrusions,
        'correction_coverage': correction_coverage,
        'landing_error_m': landing_error_m,
        'landed': landed,
        'pass_fail': pass_fail,
        'passed': all(pass_fail.values()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--mission-file',
        default='systems/aerostack2_gazebo/missions/wilderness_missions.json')
    parser.add_argument('--mission-id')
    parser.add_argument(
        '--telemetry',
        help='JSON file with {"samples": [...]} or a raw sample list')
    args = parser.parse_args()

    mission_file, missions = _load_mission(args.mission_file, args.mission_id)
    errors = _validate_contract(mission_file, missions)
    if errors:
        raise SystemExit('invalid mission contract: ' + '; '.join(errors))

    if not args.telemetry:
        print(json.dumps({
            'mission_file': args.mission_file,
            'missions': [mission['id'] for mission in missions],
            'contract_valid': True,
        }, indent=2))
        return

    telemetry = json.loads(Path(args.telemetry).read_text())
    samples = telemetry.get(
        'samples', telemetry if isinstance(telemetry, list) else [])
    results = [
        _score_mission(mission_file, mission, samples)
        for mission in missions]
    print(json.dumps({
        'results': results,
        'passed': all(result['passed'] for result in results),
    }, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Score an Aerodrone GPS-denied SITL run (tools/gps_denied_sitl_test.sh).

Reads the telemetry written by tools/gps_denied_sitl_mission.py and checks,
over the GPS-denied phase:

- horizontal error of the fused navigation fix (/navigation/fix) against
  simulator ground truth: RMSE, 95th percentile and maximum;
- the same for ArduPilot's own EKF position (what the autopilot flies on);
- that the vehicle stayed armed in GUIDED for the whole denied phase and the
  autopilot raised no EKF failsafe / GPS glitch;
- how much of the denied phase had an active absolute correction;
- that the denied phase lasted as long as configured.

A run with GPS_DENIED_DENIED_S=0 is a normal flight on GPS instead: it must
arm, take off and finish the mission in GUIDED with no pre-arm, GPS 2 or
failsafe text, and its fused/EKF errors are scored over the whole flight.

Thresholds are parameters. The defaults are the pass criteria (fused error
RMSE 25 m, p95 45 m, max 60 m, at least 240 s denied, 100 % GUIDED; see
tools/gps_denied_sitl_test.md).
Exit status 0 = pass, 1 = fail, 2 = unusable telemetry.
"""
import argparse
import json
import math
import re
import sys

EARTH_RADIUS_M = 6378137.0
FAILSAFE_PATTERN = re.compile(
    r'EKF Failsafe|EKF variance|GPS Glitch|failsafe|Lost GPS|Bad position',
    re.IGNORECASE)
# Normal flight: GPS_INPUT corrections must not get in the way of arming.
NORMAL_FLIGHT_PATTERN = re.compile(
    FAILSAFE_PATTERN.pattern + r'|PreArm|Arm: |GPS 2', re.IGNORECASE)


def horizontal_error_m(a, b):
    """Horizontal distance between two [lat, lon, ...] points, or None."""
    if not a or not b:
        return None
    values = (a[0], a[1], b[0], b[1])
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        return None
    lat0 = math.radians((a[0] + b[0]) / 2.0)
    north = math.radians(b[0] - a[0]) * EARTH_RADIUS_M
    east = math.radians(b[1] - a[1]) * EARTH_RADIUS_M * math.cos(lat0)
    return math.hypot(east, north)


def error_stats(errors, expected):
    """RMSE/p95/max of the errors; missing samples count as unscorable."""
    valid = sorted(e for e in errors if e is not None)
    stats = {'samples': len(errors), 'scored': len(valid),
             'unscored': expected - len(valid)}
    if not valid:
        return stats
    rank = max(0, math.ceil(0.95 * len(valid)) - 1)
    stats.update(
        rmse_m=round(math.sqrt(sum(e * e for e in valid) / len(valid)), 2),
        p95_m=round(valid[rank], 2),
        max_m=round(valid[-1], 2),
        final_m=round(next(e for e in reversed(errors) if e is not None), 2))
    return stats


def score_normal_flight(telemetry, thresholds):
    """GPS never denied: armed in GUIDED for the whole mission, no pre-arm,
    GPS 2 or failsafe text from arming on (pre-arm texts while the EKF is
    still initialising are normal), and the fused/EKF errors over the
    flight."""
    samples = telemetry.get('samples', [])
    events = [e.get('event', '') for e in telemetry.get('events', [])]
    ready_t = next((e['t'] for e in telemetry.get('events', [])
                    if e.get('event') == 'ready'), 0.0)
    texts = [t['text'] for t in telemetry.get('status_texts', [])
             if NORMAL_FLIGHT_PATTERN.search(t.get('text', ''))
             and t.get('t', 0.0) >= ready_t]
    fused = error_stats(
        [horizontal_error_m(s.get('truth'), s.get('fused')) for s in samples],
        len(samples))
    ekf = error_stats(
        [horizontal_error_m(s.get('truth'), s.get('ekf')) for s in samples],
        len(samples))
    guided = sum(1 for s in samples if s.get('mode') == 'GUIDED' and s.get('armed'))
    metrics = {
        'flight_samples': len(samples),
        'fused': fused,
        'ekf': ekf,
        'guided_fraction': round(guided / len(samples), 3) if samples else 0.0,
        'waypoints_reached': max((s.get('waypoint', 0) for s in samples), default=0),
        'flagged_texts': texts,
        'left_guided': [e for e in events if e.startswith('left_guided')],
        'completed': bool(telemetry.get('completed')),
        'error': telemetry.get('error'),
    }
    t = thresholds
    checks = {
        'completed': metrics['completed'] and not metrics['error'],
        'took_off': 'at_altitude' in events,
        'stayed_guided': metrics['guided_fraction'] >= t['min_guided_fraction'],
        'no_prearm_gps2_or_failsafe_text': len(texts) <= t['max_failsafe_events'],
        'fused_scored': fused.get('unscored', 1) <= t['max_unscored_samples'],
        'fused_rmse': fused.get('rmse_m', math.inf) <= t['max_fused_rmse_m'],
        'ekf_rmse': ekf.get('rmse_m', math.inf) <= t['max_ekf_rmse_m'],
    }
    return {'mode': 'normal_flight', 'metrics': metrics, 'thresholds': t,
            'checks': checks, 'passed': bool(samples) and all(checks.values())}


def score(telemetry, thresholds):
    if (telemetry.get('config') or {}).get('denied_s', 1) == 0:
        return score_normal_flight(telemetry, thresholds)
    samples = telemetry.get('samples', [])
    denied = [s for s in samples if s.get('phase') == 'denied']
    events = [e.get('event', '') for e in telemetry.get('events', [])]
    denied_start = next((e['t'] for e in telemetry.get('events', [])
                         if e.get('event') == 'gps_disabled'), None)
    denied_end = next((e['t'] for e in telemetry.get('events', [])
                       if e.get('event') == 'gps_restored'), None)
    failsafes = [
        t for t in telemetry.get('status_texts', [])
        if FAILSAFE_PATTERN.search(t.get('text', ''))
        and denied_start is not None and t.get('t', -1) >= denied_start
        and (denied_end is None or t.get('t', 0) <= denied_end)]

    fused = error_stats(
        [horizontal_error_m(s.get('truth'), s.get('fused')) for s in denied], len(denied))
    ekf = error_stats(
        [horizontal_error_m(s.get('truth'), s.get('ekf')) for s in denied], len(denied))
    guided = sum(1 for s in denied if s.get('mode') == 'GUIDED' and s.get('armed'))
    corrected = sum(
        1 for s in denied
        if s.get('correction_source') not in (None, '', 'dead_reckoning'))
    duration = (denied[-1]['t'] - denied[0]['t']) if len(denied) > 1 else 0.0
    metrics = {
        'denied_samples': len(denied),
        'denied_duration_s': round(duration, 1),
        'fused': fused,
        'ekf': ekf,
        'guided_fraction': round(guided / len(denied), 3) if denied else 0.0,
        'correction_coverage': round(corrected / len(denied), 3) if denied else 0.0,
        'failsafe_texts': [t['text'] for t in failsafes],
        'left_guided': [e for e in events if e.startswith('left_guided')],
        'completed': bool(telemetry.get('completed')),
        'error': telemetry.get('error'),
    }
    t = thresholds
    checks = {
        'completed': metrics['completed'] and not metrics['error'],
        'denied_duration': duration >= t['min_denied_s'],
        'stayed_guided': metrics['guided_fraction'] >= t['min_guided_fraction'],
        'no_failsafe': len(failsafes) <= t['max_failsafe_events'],
        'fused_scored': fused.get('unscored', 1) <= t['max_unscored_samples'],
        'fused_rmse': fused.get('rmse_m', math.inf) <= t['max_fused_rmse_m'],
        'fused_p95': fused.get('p95_m', math.inf) <= t['max_fused_p95_m'],
        'fused_max': fused.get('max_m', math.inf) <= t['max_fused_max_m'],
        'ekf_rmse': ekf.get('rmse_m', math.inf) <= t['max_ekf_rmse_m'],
        'correction_coverage': metrics['correction_coverage'] >= t['min_correction_coverage'],
    }
    return {'metrics': metrics, 'thresholds': t, 'checks': checks,
            'passed': bool(denied) and all(checks.values())}


# Pass criteria (final; tools/gps_denied_sitl_test.md explains the numbers).
DEFAULTS = {
    'min_denied_s': 240.0,
    'min_guided_fraction': 1.0,
    'max_failsafe_events': 0,
    'max_unscored_samples': 5,
    'max_fused_rmse_m': 25.0,
    'max_fused_p95_m': 45.0,
    'max_fused_max_m': 60.0,
    'max_ekf_rmse_m': 25.0,
    'min_correction_coverage': 0.8,
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('telemetry')
    for name, value in DEFAULTS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=type(value), default=value)
    args = parser.parse_args(argv)
    try:
        with open(args.telemetry) as stream:
            telemetry = json.load(stream)
    except (OSError, ValueError) as exc:
        print(f'unusable telemetry: {exc}', file=sys.stderr)
        return 2
    thresholds = {name: getattr(args, name) for name in DEFAULTS}
    result = score(telemetry, thresholds)
    print(json.dumps(result, indent=2))
    print('GPS-DENIED SITL ' + ('PASS' if result['passed'] else 'FAIL'))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

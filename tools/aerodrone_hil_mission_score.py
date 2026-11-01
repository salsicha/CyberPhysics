#!/usr/bin/env python3
"""Score a wilderness mission from a gps_denial_recorder recording
(tools/aerodrone_hil_test.md, "Mission"): the mission tree's result, every
commanded waypoint's closest approach in simulator truth, and the timing.

Waypoints are the changes of the MAVROS position setpoint the recorder logs
(the mission's take-off and GoTo targets, local ENU about ArduPilot's EKF
origin); the truth (ENU about the recorder's projection origin) is moved to
that frame with the recorded EKF origin, its height referred to where the
vehicle rested before arming (the EKF origin's altitude). A waypoint's window
runs from its setpoint to the next one (the last: to the landing's disarm).

  python3 tools/aerodrone_hil_mission_score.py RUN_DIR [--tree-log FILE] [--tree-exit N]

Writes nothing; prints the score as JSON. Exit status: 0 the tree succeeded,
every waypoint was reached within --reach-m and the vehicle landed and
disarmed; 1 otherwise; 2 unusable recording.
"""
import argparse
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_denied_sim_scorer import (
    SimClock,
    find,
    latlon_to_local,
    percentile,
    read_jsonl,
)

# BT::StdCoutLogger: "[t]: <node name>  <from> -> <to>"
TRANSITION = re.compile(r'\]:\s+(\S+)\s+(IDLE|RUNNING|SUCCESS|FAILURE)\s*->\s*'
                        r'(IDLE|RUNNING|SUCCESS|FAILURE)\b')
ANSI = re.compile(r'\x1b\[[0-9;]*m')


class Unusable(Exception):
    pass


def tree_transitions(lines):
    """{node: {'SUCCESS': n, 'FAILURE': n}} and the last result of any node
    named like the root (the last SUCCESS/FAILURE transition seen)."""
    counts, last = {}, None
    for line in lines:
        match = TRANSITION.search(ANSI.sub('', line))
        if not match:
            continue
        name, _, new = match.groups()
        if new in ('SUCCESS', 'FAILURE'):
            counts.setdefault(name, {'SUCCESS': 0, 'FAILURE': 0})[new] += 1
            last = (name, new)
    return counts, last


def score_mission(rows, tree_lines=(), tree_exit=None, reach_m=3.0):
    by_kind = {}
    for row in rows:
        by_kind.setdefault(row.get('k'), []).append(row)
    for kind_rows in by_kind.values():
        kind_rows.sort(key=lambda r: r.get('rx', 0.0))
    meta = (by_kind.get('meta') or [{}])[0]
    if 'origin_lat' not in meta:
        raise Unusable('no meta row with the projection origin')
    truth = [r for r in by_kind.get('truth', []) if all(k in r for k in ('e', 'n', 'u'))]
    origins = by_kind.get('origin', [])
    if len(truth) < 2 or not origins:
        raise Unusable('no truth or no EKF origin in the recording')
    origin = origins[-1]
    oe, on = latlon_to_local(origin['lat'], origin['lon'],
                             float(meta['origin_lat']), float(meta['origin_lon']))
    states = by_kind.get('state', [])
    armed_at = next((s['rx'] for s in states if s.get('armed')), None)
    disarmed_at = None
    if armed_at is not None:
        disarmed_at = next((s['rx'] for s in states if s['rx'] > armed_at and not s.get('armed')),
                           None)
    resting = [r['u'] for r in truth if armed_at is None or r['rx'] < armed_at][-50:]
    u_home = sorted(resting)[len(resting) // 2] if resting else truth[0]['u']
    clock = SimClock([(r['rx'], r['sim']) for r in by_kind.get('clock', [])])
    setpoints = [s for s in by_kind.get('setpoint', []) if all(k in s for k in ('x', 'y', 'z'))]
    end = disarmed_at if disarmed_at is not None else truth[-1]['rx']
    waypoints = []
    for i, sp in enumerate(setpoints):
        start = sp['rx']
        stop = setpoints[i + 1]['rx'] if i + 1 < len(setpoints) else end
        best_h = best_3d = math.inf
        reached = None
        for r in truth:
            if r['rx'] < start:
                continue
            if r['rx'] >= stop:
                break
            de, dn = r['e'] - oe - sp['x'], r['n'] - on - sp['y']
            du = r['u'] - u_home - sp['z']
            h = math.hypot(de, dn)
            d3 = math.sqrt(h * h + du * du)
            best_h, best_3d = min(best_h, h), min(best_3d, d3)
            if reached is None and d3 <= reach_m:
                reached = r['rx']
        waypoints.append({
            'target': [round(sp['x'], 1), round(sp['y'], 1), round(sp['z'], 1)],
            'closest_horizontal_m': round(best_h, 2) if math.isfinite(best_h) else None,
            'closest_3d_m': round(best_3d, 2) if math.isfinite(best_3d) else None,
            'reached': reached is not None,
            'to_reach_s': round(reached - start, 1) if reached is not None else None,
            'to_reach_sim_s': (round(clock(reached) - clock(start), 1)
                               if reached is not None else None),
            'window_s': round(stop - start, 1)})
    final = [r for r in truth if disarmed_at is not None and r['rx'] >= disarmed_at][:20]
    landed = bool(final) and max(abs(r['u'] - u_home) for r in final) < 1.5
    counts, last = tree_transitions(tree_lines)
    horizontal = [w['closest_horizontal_m'] for w in waypoints if w['closest_horizontal_m'] is not None]
    three_d = [w['closest_3d_m'] for w in waypoints if w['closest_3d_m'] is not None]
    result = {
        'tree_exit': tree_exit,
        'tree_last_transition': last,
        'tree_transitions': counts,
        'waypoints': len(waypoints),
        'waypoints_reached': sum(w['reached'] for w in waypoints),
        'closest_horizontal_m': ({'median': round(percentile(sorted(horizontal), 50), 2),
                                  'max': round(max(horizontal), 2)} if horizontal else None),
        'closest_3d_m': ({'median': round(percentile(sorted(three_d), 50), 2),
                          'max': round(max(three_d), 2)} if three_d else None),
        'armed_to_disarmed_s': (round(disarmed_at - armed_at, 1)
                                if armed_at is not None and disarmed_at is not None else None),
        'armed_to_disarmed_sim_s': (round(clock(disarmed_at) - clock(armed_at), 1)
                                    if armed_at is not None and disarmed_at is not None else None),
        'landed_and_disarmed': landed,
        'per_waypoint': waypoints,
    }
    ok = (tree_exit in (0, None) and (last is None or last[1] == 'SUCCESS') and
          bool(waypoints) and result['waypoints_reached'] == len(waypoints) and landed)
    if tree_exit is None and last is None:
        ok = False  # nothing says the tree finished
    result['pass'] = ok
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('run_dir')
    parser.add_argument('--tree-log', help="the mission tree container's log")
    parser.add_argument('--tree-exit', type=int, help="the mission tree container's exit code")
    parser.add_argument('--reach-m', type=float, default=3.0,
                        help='a waypoint is reached within this 3D distance in truth '
                             "(the stacks' go_to_threshold)")
    args = parser.parse_args(argv)
    path = find(args.run_dir, 'recording.jsonl')
    if path is None:
        print(f'no recording.jsonl in {args.run_dir}', file=sys.stderr)
        return 2
    lines = []
    if args.tree_log and os.path.exists(args.tree_log):
        with open(args.tree_log, encoding='utf-8', errors='replace') as stream:
            lines = stream.readlines()
    try:
        result = score_mission(read_jsonl(path), lines, args.tree_exit, args.reach_m)
    except Unusable as error:
        print(f'unusable: {error}', file=sys.stderr)
        return 2
    print(json.dumps(result, indent=1))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())

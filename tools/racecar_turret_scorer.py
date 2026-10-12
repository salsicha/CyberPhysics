#!/usr/bin/env python3
"""Score RACECAR Turret Isaac runs: driving and aiming at the same time
(systems/racecar_turret/interfaces.md, "Isaac evaluation";
tools/racecar_turret_isaac_test.sh runs it on the recordings of
tools/racecar_turret_evaluator.py).

Per run, over the course (Nav2 goal sent to its result):

* driving: course completed (NavigateThroughPoses succeeded), time,
  collisions (the simulator's count, /simulation/truth), distance and mean
  speed of the true path, and the SLAM pose's error against the truth;
* aiming, from the truth: the angle between the turret camera's optical
  axis and its line of sight to the selected target's centroid (mean, p95,
  fraction under on_target_deg), over the samples where the target can be
  aimed at at all ("trackable": its line of sight is clear and it is within
  the turret's pan and tilt range), and over every sample; the fraction of
  trackable samples /turret/status reports `locked` (and aiming at it
  rather than at another person); every loss of the target (occluded or out
  of reach for at least min_loss_s) and how long the turret took after it
  to aim within on_target_deg again (reacquisition) and to report `locked`;
  how often the evaluator, as the operator, had to take over and select the
  target again (`reselected` events: the tracker lost it or followed
  another person); and the same aim statistics over the samples where the
  tracker follows the target (its selected detection contains the target's
  projected centroid: "following"), which measure the aiming loop apart
  from the tracker's identity switches.

With several runs (--run label=dir ...) the thresholds of
systems/racecar_turret/validation/acceptance_thresholds.json apply to each
run's driving and, for the runs it names in aiming_runs, to aiming; the
summary also compares the runs. Plain Python (no numpy):
tests/test_racecar_turret_scorer.py checks it.

Exit status: 0 every threshold met, 1 one missed, 2 a run is unusable.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
import gzip
import json
import math
from pathlib import Path
import sys

TRUTH = '/simulation/truth'
STATUS = '/turret/status'
SEGMENTS = '/turret/yolo/segments'
PAN_LIMIT = 1.5708
TILT_LIMITS = (-0.65, 0.75)


def read_recording(path):
    """Rows of a recording (.jsonl or .jsonl.gz); a truncated last line is
    skipped."""
    path = Path(path)
    if not path.is_file() and Path(f'{path}.gz').is_file():
        path = Path(f'{path}.gz')
    opener = gzip.open if path.suffix == '.gz' else open
    rows = []
    with opener(path, 'rt', encoding='utf-8') as handle:
        for line in handle:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def percentile(values, fraction):
    """Linear-interpolated percentile (fraction in [0, 1])."""
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def mean(values):
    return sum(values) / len(values) if values else None


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def project(truth, point, intrinsics):
    """Pixel (u, v) of a point in the turret image of a truth sample, or
    None behind the camera (tools/racecar_turret_evaluator.project_truth)."""
    camera = truth['turret_camera']
    right = _cross(camera['forward'], camera['up'])
    delta = _sub(point, camera['position'])
    depth = _dot(delta, camera['forward'])
    if depth <= 1e-6:
        return None
    fx, fy, cx, cy = intrinsics
    return cx + fx * _dot(delta, right) / depth, cy - fy * _dot(delta, camera['up']) / depth


def angle_deg(forward, sight):
    norm = math.sqrt(sum(v * v for v in forward)) * math.sqrt(sum(v * v for v in sight))
    if norm <= 0.0:
        return float('nan')
    cosine = sum(a * b for a, b in zip(forward, sight)) / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def aim_error_deg(truth, target):
    camera = truth['turret_camera']
    return angle_deg(camera['forward'], _sub(target['centroid'], camera['position']))


def reachable(truth, target, pan_limit=PAN_LIMIT, tilt_limits=TILT_LIMITS):
    """Whether the turret can point at the target: the pan and tilt that aim
    from the camera at its centroid, relative to the car, are within the
    joint limits."""
    camera = truth['turret_camera']['position']
    dx, dy, dz = _sub(target['centroid'], camera)
    bearing = math.atan2(dy, dx) - float(truth['vehicle']['yaw'])
    pan = -math.atan2(math.sin(bearing), math.cos(bearing))  # positive right
    tilt = math.atan2(dz, math.hypot(dx, dy))
    return abs(pan) <= pan_limit and tilt_limits[0] <= tilt <= tilt_limits[1]


class Run:
    """A recording split into its events and the samples of the course."""

    def __init__(self, rows, target=None):
        self.events = [row for row in rows if 'event' in row]
        by_name = {}
        for event in self.events:
            by_name.setdefault(event['event'], event)
        self.event = by_name
        selected = by_name.get('selected') or by_name.get('start') or {}
        self.target = target or selected.get('target') or 'person_1'
        self.start = by_name['goal_sent']['t'] if 'goal_sent' in by_name else None
        result = by_name.get('result')
        self.end = result['t'] if result else None
        self.result = result
        topics = {}
        for row in rows:
            if 'topic' in row:
                topics.setdefault(row['topic'], []).append(row)
        self.topics = topics
        self.truth = [row for row in topics.get(TRUTH, []) if self._in_course(row['data']['stamp'])]
        self.reselections = [event for event in self.events
                             if event['event'] == 'reselected' and self._in_course(event['t'])]
        self.status = topics.get(STATUS, [])

    def _in_course(self, t):
        return (self.start is not None and t >= self.start
                and (self.end is None or t <= self.end))

    def usable(self):
        return self.start is not None and len(self.truth) >= 2


def status_at(status_rows, t):
    """The last /turret/status received at or before simulation time t."""
    index = bisect_right([row['t'] for row in status_rows], t) - 1
    return status_rows[index]['data'] if index >= 0 else None


def following_flags(run, intrinsics, margin=1.3, pad_px=10.0):
    """[(receive time, flag)] per /turret/yolo/segments message: whether the
    tracker's selected detection holds the target, i.e. contains the target's
    centroid projected into that frame (truth at the frame's stamp); None
    when no detection is selected in that frame."""
    truth_times = [row['data']['stamp'] for row in run.truth]
    status_times = [row['t'] for row in run.status]
    flags = []
    for row in run.topics.get(SEGMENTS, []):
        data = row['data']
        detections = data.get('detections', [])
        selected = [d for d in detections if d.get('selected')]
        if not selected:
            index = bisect_right(status_times, row['t']) - 1
            wanted = str(run.status[index]['data'].get('target_id') or '') if index >= 0 else ''
            selected = [d for d in detections if wanted and str(d.get('id')) == wanted]
        stamp = data.get('stamp') or row['t']
        index = bisect_right(truth_times, stamp)
        nearest = min((i for i in (index - 1, index) if 0 <= i < len(truth_times)),
                      key=lambda i: abs(truth_times[i] - stamp), default=None)
        if not selected or nearest is None or abs(truth_times[nearest] - stamp) > 0.1:
            flags.append((row['t'], None))
            continue
        truth = run.truth[nearest]['data']
        target = next((t for t in truth['targets'] if t['name'] == run.target), None)
        pixel = project(truth, target['centroid'], intrinsics) if target else None
        x, y, w, h = (float(v) for v in selected[0]['bbox_xywh'])
        flags.append((row['t'], pixel is not None
                      and abs(pixel[0] - x) <= w / 2.0 * margin + pad_px
                      and abs(pixel[1] - y) <= h / 2.0 * margin + pad_px))
    return flags


def following_at(flags, t, hold_s=1.0):
    """Whether the tracker follows the target at time t: the last frame
    with a selected detection, if it is at most hold_s old."""
    index = bisect_right([time for time, _ in flags], t) - 1
    while index >= 0 and t - flags[index][0] <= hold_s:
        if flags[index][1] is not None:
            return flags[index][1]
        index -= 1
    return False


def driving_metrics(run):
    truth = run.truth
    first, last = truth[0]['data'], truth[-1]['data']
    end = run.end if run.end is not None else last['stamp']
    duration = end - run.start
    distance = 0.0
    for previous, current in zip(truth, truth[1:]):
        a, b = previous['data']['vehicle'], current['data']['vehicle']
        distance += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
    slam_errors = []
    truth_times = [row['data']['stamp'] for row in truth]
    for row in run.topics.get('slam_pose', []):
        pose = row['data']
        if not run._in_course(pose['stamp']):
            continue
        index = min(range(len(truth_times)), key=lambda i: abs(truth_times[i] - pose['stamp']))
        if abs(truth_times[index] - pose['stamp']) <= 0.1:
            vehicle = truth[index]['data']['vehicle']
            slam_errors.append(math.hypot(pose['x'] - vehicle['x'], pose['y'] - vehicle['y']))
    status = run.result.get('status_name') if run.result else 'no result'
    return {
        'completed': bool(run.result and run.result.get('status') == 4),
        'result': status,
        'time_s': round(duration, 2),
        'collisions': int(last['collisions']) - int(first['collisions']),
        'distance_m': round(distance, 2),
        'mean_speed_mps': round(distance / duration, 3) if duration > 0 else None,
        'slam_error_mean_m': _round(mean(slam_errors), 3),
        'slam_error_max_m': _round(max(slam_errors) if slam_errors else None, 3),
    }


def _round(value, digits=2):
    return None if value is None else round(value, digits)


def losses(samples, min_loss_s):
    """(start, end, cause) of every interval the target was not trackable
    for at least min_loss_s; end is None when it never came back."""
    intervals = []
    start = cause = None
    for t, trackable, why in samples:
        if not trackable and start is None:
            start, cause = t, why
        elif trackable and start is not None:
            if t - start >= min_loss_s:
                intervals.append((start, t, cause))
            start = None
    if start is not None and samples and samples[-1][0] - start >= min_loss_s:
        intervals.append((start, None, cause))
    return intervals


def aiming_metrics(run, on_target_deg=5.0, min_loss_s=0.5, other_margin_deg=2.0):
    samples = []
    errors_all, errors_trackable, errors_following = [], [], []
    locked_trackable = on_other = trackable_count = 0
    ready = run.event.get('ready') or {}
    flags = following_flags(run, ready['intrinsics']) if ready.get('intrinsics') else None
    for row in run.truth:
        truth = row['data']
        targets = {target['name']: target for target in truth['targets']}
        target = targets.get(run.target)
        if target is None:
            continue
        error = aim_error_deg(truth, target)
        visible = bool(target.get('visible', True))
        in_reach = reachable(truth, target)
        trackable = visible and in_reach
        cause = None if trackable else ('occluded' if not visible else 'out_of_reach')
        status = status_at(run.status, truth['stamp']) or {}
        others = [aim_error_deg(truth, other) for name, other in targets.items()
                  if name != run.target and other.get('visible', True)]
        aimed_at_other = bool(others) and min(others) < on_target_deg and min(others) + other_margin_deg < error
        samples.append((truth['stamp'], trackable, cause, error, bool(status.get('locked')),
                        aimed_at_other))
        errors_all.append(error)
        if trackable:
            trackable_count += 1
            errors_trackable.append(error)
            locked_trackable += bool(status.get('locked')) and not aimed_at_other
            on_other += aimed_at_other
            if flags is not None and following_at(flags, row['t']):
                errors_following.append(error)
    if not samples:
        return None
    events = []
    unrecovered = 0
    for start, end, cause in losses([(s[0], s[1], s[2]) for s in samples], min_loss_s):
        event = {'start_s': round(start - run.start, 2), 'cause': cause,
                 'lost_s': round((end if end is not None else samples[-1][0]) - start, 2),
                 'returned': end is not None, 'reacquire_s': None, 'relock_s': None}
        if end is not None:
            # Until the target is lost again (or the course ends).
            for t, trackable, _, error, locked, other in (s for s in samples if s[0] >= end):
                if not trackable:
                    break
                if event['reacquire_s'] is None and error < on_target_deg and not other:
                    event['reacquire_s'] = round(t - end, 2)
                if event['relock_s'] is None and locked and not other:
                    event['relock_s'] = round(t - end, 2)
                if event['reacquire_s'] is not None and event['relock_s'] is not None:
                    break
            unrecovered += event['reacquire_s'] is None
        events.append(event)
    reacquired = [e['reacquire_s'] for e in events if e['reacquire_s'] is not None]
    count = len(samples)
    return {
        'samples': count,
        'trackable_fraction': round(trackable_count / count, 3),
        'mean_error_deg': _round(mean(errors_trackable)),
        'p95_error_deg': _round(percentile(errors_trackable, 0.95)),
        'on_target_fraction': _round(sum(e < on_target_deg for e in errors_trackable)
                                     / len(errors_trackable), 3) if errors_trackable else None,
        'locked_fraction': _round(locked_trackable / trackable_count, 3) if trackable_count else None,
        'aimed_at_other_fraction': _round(on_other / trackable_count, 3) if trackable_count else None,
        'all_samples': {
            'mean_error_deg': _round(mean(errors_all)),
            'p95_error_deg': _round(percentile(errors_all, 0.95)),
            'on_target_fraction': round(sum(e < on_target_deg for e in errors_all) / count, 3),
            'locked_fraction': round(sum(s[4] for s in samples) / count, 3),
        },
        'losses': events,
        'reacquisitions': len(reacquired),
        'mean_reacquire_s': _round(mean(reacquired)),
        'max_reacquire_s': _round(max(reacquired) if reacquired else None),
        'unrecovered_losses': unrecovered,
        'following_fraction': (_round(len(errors_following) / trackable_count, 3)
                               if flags is not None and trackable_count else None),
        'following_mean_error_deg': _round(mean(errors_following)),
        'following_p95_error_deg': _round(percentile(errors_following, 0.95)),
        'following_on_target_fraction': _round(sum(e < on_target_deg for e in errors_following)
                                               / len(errors_following), 3) if errors_following else None,
        'reselections': len(run.reselections),
        'reselection_reasons': [event.get('reason') for event in run.reselections],
    }


def score_run(rows, target=None, on_target_deg=5.0, min_loss_s=0.5):
    run = Run(rows, target)
    if not run.usable():
        return None
    return {'target': run.target, 'driving': driving_metrics(run),
            'aiming': aiming_metrics(run, on_target_deg, min_loss_s)}


CHECKS = {
    # name: (section, metric, comparison)
    'course_completed': ('driving', 'completed', 'true'),
    'max_course_time_s': ('driving', 'time_s', 'max'),
    'max_collisions': ('driving', 'collisions', 'max'),
    'min_mean_speed_mps': ('driving', 'mean_speed_mps', 'min'),
    'min_trackable_fraction': ('aiming', 'trackable_fraction', 'min'),
    'max_mean_aim_error_deg': ('aiming', 'mean_error_deg', 'max'),
    'max_p95_aim_error_deg': ('aiming', 'p95_error_deg', 'max'),
    'min_on_target_fraction': ('aiming', 'on_target_fraction', 'min'),
    'min_locked_fraction': ('aiming', 'locked_fraction', 'min'),
    'max_aimed_at_other_fraction': ('aiming', 'aimed_at_other_fraction', 'max'),
    'max_reacquire_time_s': ('aiming', 'max_reacquire_s', 'max'),
    'max_unrecovered_losses': ('aiming', 'unrecovered_losses', 'max'),
    'max_reselections': ('aiming', 'reselections', 'max'),
    'min_following_fraction': ('aiming', 'following_fraction', 'min'),
    'max_following_mean_error_deg': ('aiming', 'following_mean_error_deg', 'max'),
    'max_following_p95_error_deg': ('aiming', 'following_p95_error_deg', 'max'),
    'min_following_on_target_fraction': ('aiming', 'following_on_target_fraction', 'min'),
}


def check(score, thresholds, aiming=True):
    """[(name, passed, value, limit)] of the thresholds that apply."""
    results = []
    for name, (section, metric, kind) in CHECKS.items():
        if name not in thresholds or (section == 'aiming' and not aiming):
            continue
        limit = thresholds[name]
        value = (score.get(section) or {}).get(metric)
        if kind == 'true':
            passed = value is True or not limit
        elif value is None:
            # No reacquisition at all is fine; anything else missing fails.
            passed = name == 'max_reacquire_time_s'
        elif kind == 'max':
            passed = value <= limit
        else:
            passed = value >= limit
        results.append((name, passed, value, limit))
    return results


def compare(scores):
    """Aiming metrics side by side, with the difference of each run from
    the first."""
    labels = list(scores)
    metrics = ('mean_error_deg', 'p95_error_deg', 'on_target_fraction', 'locked_fraction',
               'following_fraction', 'following_mean_error_deg', 'following_p95_error_deg',
               'following_on_target_fraction', 'mean_reacquire_s', 'max_reacquire_s',
               'trackable_fraction', 'reselections')
    table = {}
    for metric in metrics:
        values = {label: (scores[label]['aiming'] or {}).get(metric) for label in labels}
        row = dict(values)
        base = values[labels[0]]
        for label in labels[1:]:
            if base is not None and values[label] is not None:
                row[f'{label}_minus_{labels[0]}'] = round(values[label] - base, 3)
        table[metric] = row
    return table


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='*', help='run directories (label: the directory name)')
    parser.add_argument('--run', action='append', default=[], metavar='LABEL=DIR',
                        help='a labelled run directory holding recording.jsonl[.gz]')
    parser.add_argument('--thresholds', default=None)
    parser.add_argument('--target', default=None, help='target name (default: the recorded one)')
    parser.add_argument('--output', default=None, help='summary JSON')
    args = parser.parse_args(argv)
    runs = [(Path(d).name, Path(d)) for d in args.runs]
    runs += [(spec.split('=', 1)[0], Path(spec.split('=', 1)[1])) for spec in args.run]
    if not runs:
        parser.error('no run given')
    thresholds = json.loads(Path(args.thresholds).read_text()) if args.thresholds else {}
    on_target = float(thresholds.get('on_target_deg', 5.0))
    min_loss = float(thresholds.get('min_loss_s', 0.5))
    aiming_runs = thresholds.get('aiming_runs', [runs[0][0]])
    summary = {'runs': {}, 'checks': {}, 'thresholds': args.thresholds}
    status = 0
    for label, directory in runs:
        score = score_run(read_recording(directory / 'recording.jsonl'), args.target,
                          on_target, min_loss)
        if score is None:
            print(f'RACECAR TURRET SCORE {label}: no usable run in {directory}')
            summary['runs'][label] = None
            status = 2
            continue
        summary['runs'][label] = score
        (directory / 'score.json').write_text(json.dumps(score, indent=2) + '\n')
        results = check(score, thresholds, aiming=label in aiming_runs)
        summary['checks'][label] = [{'check': n, 'passed': p, 'value': v, 'limit': lim}
                                    for n, p, v, lim in results]
        driving, aiming = score['driving'], score['aiming'] or {}
        print(f'RACECAR TURRET SCORE {label}: course {driving["result"]} in {driving["time_s"]} s, '
              f'{driving["collisions"]} collisions, {driving["mean_speed_mps"]} m/s; aim mean '
              f'{aiming.get("mean_error_deg")} p95 {aiming.get("p95_error_deg")} deg, on target '
              f'{aiming.get("on_target_fraction")}, locked {aiming.get("locked_fraction")}, '
              f'trackable {aiming.get("trackable_fraction")}, reacquire max '
              f'{aiming.get("max_reacquire_s")} s ({aiming.get("unrecovered_losses")} unrecovered), '
              f'{aiming.get("reselections")} reselections; following '
              f'{aiming.get("following_fraction")}: aim mean {aiming.get("following_mean_error_deg")} '
              f'p95 {aiming.get("following_p95_error_deg")} deg, on target '
              f'{aiming.get("following_on_target_fraction")}')
        for name, passed, value, limit in results:
            if not passed:
                print(f'  FAIL {name}: {value} (limit {limit})')
                status = max(status, 1)
    usable = {label: score for label, score in summary['runs'].items() if score}
    if len(usable) > 1:
        summary['comparison'] = compare(usable)
    summary['passed'] = status == 0
    text = json.dumps(summary, indent=2) + '\n'
    if args.output:
        Path(args.output).write_text(text)
    print(text if len(usable) > 1 else '', end='')
    return status


if __name__ == '__main__':
    sys.exit(main())

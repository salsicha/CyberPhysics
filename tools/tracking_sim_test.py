#!/usr/bin/env python3
"""Fly and score the Aerodrone object-tracking test in a simulator
(systems/aerodrone/tracking.md; tools/tracking_sim_test.sh runs it).

Runs in the stack (tools/tracking_sim_test.yaml) with the `tracking`
profile (``--detector real``: the YOLO detector on the simulated camera,
end to end) or `tracking_truth` (``--detector truth``: the ground-truth
detector's boxes, which tests the tracker's geolocation, filter and
control apart from detection), against the simulator's targets
(/simulation/targets).

Like the operator in the web UI, the test picks a target by the ``id`` of
its box. It pairs every detection frame with the truth boxes of the same
camera frame (the truth detector projecting /simulation/targets, which
the test's compose layer runs beside the real detector on
/tracking/truth/detections; with ``--detector truth`` the detections are
the truth boxes, named after the targets) and gives each target the track
whose box centre is nearest its own (``associate``). A target's id is the
one it has had in the last three frames.

``fly``: arm, offboard and take off through Aerostack2, fly a GoTo above
the target, then (each step logged to the recording):

1. ``preempt_goto``: pick --target, start a far GoTo and send a centered
   track command while it runs: the GoTo must end and the tracker follow;
2. ``centered``: follow --target at --altitude m for --centered-s;
3. ``slant``: --slant-range m at --depression deg for --slant-s;
4. ``lost``: track --lost-target centered at --lost-altitude m, hide it
   under the terrain
   (``/simulation/targets/hide``; ``/tracking/truth/hide`` for the truth
   detector's own targets) for --hidden-s: state lost and hovering; once
   shown, pick it again (the detector gives it a new id once it has been
   gone for its max_lost_s, 2 s) and track it;
5. ``stop``: ``stop`` must leave the aircraft idle and hovering;
6. ``preempt_map_goal``: track again, then publish a map goal: idle;
   ``preempt_goto_behavior``: track again, then a GoTo action: idle;
7. land.

While following, a target the tracker has lost under a new id that it
did not take over itself (aerodrone_tracker's track handover) is tracked
again under that id --repick-s later, as the operator would click its new
box; one out of view for 10 s is found again from above its route. Those
times are simulation seconds, like the tracker's lost_timeout_s (the
simulators may run slower than real time). A hold above a route is above
the route's point nearest its middle, which the target passes every lap.

Records 10 Hz rows (JSON lines): simulation time, the aircraft's navigation
position and yaw and its simulator ground truth in the navigation frame
(and its true roll and pitch, which move a body-fixed camera's view),
the targets' truth, the tracker's status and estimate, the commanded id and
the step; and a ``det`` line per detection frame: the targets in view, the
id each was given, how many tracks matched no target and the id the
tracker followed.

``score``: per step, after a settling time, median/p95/max of the
aircraft's horizontal distance from the target (centered), of the slant
range and depression errors and the bearing error (slant), against truth
and in the navigation frame, and of the tracker's target estimate against
the truth; the fraction of the step spent tracking and the time to
tracking again after each loss; the detector's recall of the target, its
ids and id switches; the checks of the lost, stop and pre-emption steps;
the detector's frame rate (in simulation and wall time); with --cpu and
--stack-log, the host's and the containers' CPU and the detector's own
timing. Each step passes or fails against THRESHOLDS. Plain Python:
tests/test_tracking_sim_test.py checks it.
"""
import argparse
import json
import math
import re
import sys
import threading
import time

METERS_PER_DEG_LAT = 111320.0
TRUTH = {'gazebo': ('/aerodrone/ground_truth/pose', 'pose'),
         'isaac': ('/aerodrone/simulation/ground_truth/odom', 'odometry')}
# Pass/fail limits of score(). The follow limits ask that the target stays
# well inside the image and the geometry is held. The scenario's car can be
# followed: it laps a 60 m x 24 m oval at 3.5 m/s, 70 % of the aircraft's
# speed limit, with 12 m-radius turns (tracking_targets.json; the first
# scenario's car lapped a 26 m x 16 m circuit at the 5 m/s limit itself,
# and the aircraft trailed it by 11 m, median, even with the ground-truth
# detector). The narrowest simulated camera (60 deg across 640 x 480) sees
# 0.43 x the height to either side of the centre across the image: the
# centered limits are fractions of the commanded height. In slant the
# target is (90 - depression) deg off nadir along the long axis, 10 deg
# inside that camera's edge at 70 deg: a depression shallower than
# commanded moves it out, a steeper one toward the centre.
THRESHOLDS = {
    'tracking_fraction': 0.9,      # of a follow step, from its first command
    'centered_median_frac': 0.25,  # horizontal distance / commanded height
    'centered_p95_frac': 0.35,
    'range_median_m': 5.0,         # slant range error
    'range_p95_m': 12.0,
    'depression_median_deg': 8.0,  # depression error, either way
    'depression_shallow_p95_deg': 8.0,  # shallower than commanded
    'recall': 0.8,                 # frames with the followed target in view
    'max_reselections': 3,         # by the operator, per follow step
    'following': 0.8,              # frames whose target id the tracker follows
    'lost_margin_s': 1.0,          # lost within lost_timeout_s + this
    'reacquire_s': 8.0,            # target shown again -> tracking
    'hover_drift_m': 1.0,          # lost and stopped: hovering
    'idle_s': 1.0,                 # stop -> idle
    'detector_rate_hz': 4.5,       # median frame rate, simulation time
    # The fire scenario (--scenario fire): the stand-in's shot at most this
    # long after the fire command (the cycle: 0.92 s priming, 0.92 s
    # chambering, 0.17 s to the sear's release, at the ST3215's speed), and
    # the bore's true ground point at the shot within this of the car's
    # centre (the fire control's own gate is 1 m from its estimate) and
    # inside its footprint.
    'fire_shot_s': 3.5,
    'fire_offset_m': 1.5,
}
# The scenario's car (the hatchback model): its footprint, metres.
CAR_FOOTPRINT_M = (4.0, 1.7)


def latlon_to_local(lat, lon, origin_lat, origin_lon):
    """synthetic_world.latlon_to_local (kept here: this tool runs in the
    aerostack2 image and in the tests alike)."""
    scale = METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    return (lon - origin_lon) * scale, (lat - origin_lat) * METERS_PER_DEG_LAT


def local_to_latlon(east, north, origin_lat, origin_lon):
    scale = METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    return origin_lat + north / METERS_PER_DEG_LAT, origin_lon + east / scale


def world_to_navigation(east, north, world_origin, navigation_origin):
    """A simulator world point (ENU about INITIAL_LAT/INITIAL_LON) in the
    navigation frame (ENU about the EKF origin), horizontally."""
    lat, lon = local_to_latlon(east, north, *world_origin)
    return latlon_to_local(lat, lon, *navigation_origin[:2])


# --- picking targets ----------------------------------------------------------
# Classes a detection of a target may have (the detector hesitates between
# car and van); any other class only itself.
KINDS = (frozenset({'car', 'van', 'truck', 'bus'}),
         frozenset({'bicycle', 'motorcycle', 'tricycle'}))


def same_kind(a, b):
    a, b = str(a or '').lower(), str(b or '').lower()
    return a == b or any(a in kind and b in kind for kind in KINDS)


def associate(truth, detections, gate_px=25.0, gate_scale=0.6, max_size_ratio=3.0):
    """{target name: detection id}: each truth box ({'id': name, 'cls',
    'cx', 'cy', 'w', 'h'}) with the detection of its kind of object whose
    centre is nearest its own, within max(gate_px, gate_scale times the
    longer side of either box), their longer sides within max_size_ratio
    (a person beside a missed car is not the car); nearest pairs first,
    each detection once. Boxes without a class match any class."""
    pairs = []
    for target in truth:
        for detection in detections:
            if target.get('cls') and detection.get('cls') and \
                    not same_kind(target['cls'], detection['cls']):
                continue
            sides = (max(target['w'], target['h']), max(detection['w'], detection['h']))
            if max(sides) > max_size_ratio * max(min(sides), 1.0):
                continue
            distance = math.hypot(detection['cx'] - target['cx'],
                                  detection['cy'] - target['cy'])
            gate = max(gate_px, gate_scale * max(sides))
            if distance <= gate:
                pairs.append((distance, target['id'], detection['id']))
    match, used = {}, set()
    for _, name, detection in sorted(pairs):
        if name not in match and detection not in used:
            match[name] = detection
            used.add(detection)
    return match


class FramePairing:
    """Pairs each detection frame with the truth boxes of the same camera
    frame (equal stamps, ns). A detection frame whose truth has not come
    by the time the truth is max_wait_ns newer is dropped."""

    def __init__(self, keep=100, max_wait_ns=2_000_000_000):
        self.truth = {}
        self.pending = []
        self.keep = keep
        self.max_wait_ns = max_wait_ns
        self.dropped = 0

    def add_truth(self, stamp_ns, boxes):
        self.truth[stamp_ns] = boxes
        while len(self.truth) > self.keep:
            del self.truth[min(self.truth)]

    def add_detections(self, stamp_ns, boxes):
        self.pending.append((stamp_ns, boxes))

    def ready(self):
        """[(stamp_ns, detections, truth)] paired since the last call."""
        paired, left = [], []
        for stamp, boxes in self.pending:
            if stamp in self.truth:
                paired.append((stamp, boxes, self.truth[stamp]))
            else:
                left.append((stamp, boxes))
        newest = max(self.truth) if self.truth else None
        self.pending = [item for item in left
                        if newest is None or newest - item[0] <= self.max_wait_ns]
        self.dropped += len(left) - len(self.pending)
        return paired


def stable_id(frames, name, count=3):
    """The id `name` has had in each of the last `count` frames ({name:
    id} each), or None."""
    recent = frames[-count:]
    if len(recent) < count or any(not frame.get(name) for frame in recent):
        return None
    ids = {frame[name] for frame in recent}
    return ids.pop() if len(ids) == 1 else None


def tilt_deg(quaternion):
    """(roll, pitch) in degrees of an (x, y, z, w) body attitude (FLU,
    ZYX): what moves a body-fixed camera's view."""
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return (round(math.degrees(roll), 2), round(math.degrees(pitch), 2))


def route_hold(samples):
    """(east, north, up) of the route sample nearest the samples' mean: a
    point the target passes on every lap (a loop's middle may be metres from
    its route)."""
    mean = [sum(values) / len(values) for values in zip(*samples)]
    return tuple(min(samples, key=lambda sample: math.dist(sample[:2], mean[:2])))


# --- scoring ------------------------------------------------------------------
def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = fraction * (len(values) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def stats(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return None
    return {'n': len(values), 'median': round(percentile(values, 0.5), 3),
            'p95': round(percentile(values, 0.95), 3), 'max': round(max(values), 3)}


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def slant_geometry(drone, target):
    """(slant range, depression deg, bearing deg) from drone to target."""
    de, dn, du = (target[i] - drone[i] for i in range(3))
    horizontal = math.hypot(de, dn)
    return (math.sqrt(horizontal ** 2 + du ** 2), math.degrees(math.atan2(-du, horizontal)),
            math.degrees(math.atan2(dn, de)))


def row_target(row, name):
    for item in row.get('targets') or []:
        if item.get('name') == name:
            return (item['east'], item['north'], item['up'])
    return None


def state_of(row):
    return (row.get('status') or {}).get('state')


def reacquisitions(rows, start):
    """Seconds from each loss (state lost or error after tracking) to
    tracking again, from rows after start; an unrecovered loss counts to
    the last row."""
    times, lost_at, tracked = [], None, False
    for row in rows:
        if row['sim'] < start:
            continue
        state = state_of(row)
        if state == 'tracking':
            if lost_at is not None:
                times.append(row['sim'] - lost_at)
                lost_at = None
            tracked = True
        elif state in ('lost', 'error') and tracked and lost_at is None:
            lost_at = row['sim']
    if lost_at is not None:
        times.append(rows[-1]['sim'] - lost_at)
    return times


def score_ids(dets, name, start=None, end=None):
    """The detector on one target: frames it was in view, the fraction
    with a track on it (recall), the distinct ids and the id switches
    between consecutive frames that had one, and the fraction of those
    frames whose id was the one the tracker followed ('following')."""
    dets = [det for det in dets if (start is None or det['sim'] >= start)
            and (end is None or det['sim'] <= end)]
    in_view = [det for det in dets if name in (det.get('visible') or [])]
    ids = [(det.get('match') or {}).get(name) for det in in_view]
    matched = [(det, value) for det, value in zip(in_view, ids) if value]
    switches = sum(1 for (_, a), (_, b) in zip(matched, matched[1:]) if a != b)
    result = {'frames_in_view': len(in_view), 'ids': sorted({value for _, value in matched},
                                                            key=lambda v: (len(v), v)),
              'id_switches': switches,
              'recall': round(len(matched) / len(in_view), 3) if in_view else None}
    if matched:
        result['following'] = round(
            sum(det.get('cmd') == value for det, value in matched) / len(matched), 3)
    if dets:
        result['unmatched_tracks_per_frame'] = round(
            sum(det.get('fp', 0) for det in dets) / len(dets), 3)
    return result


def score_follow(rows, step, settle_s, dets=(), limits=THRESHOLDS):
    """Errors of one following step (centered or slant) while tracking,
    from settle_s after the step's first track command and after each
    re-acquisition from above the route, and its pass/fail checks."""
    rows = [row for row in rows if row.get('step') == step['name']]
    if not rows:
        return {'rows': 0, 'passed': False}
    events = [(row['sim'], row['event']['command']) for row in rows
              if 'command' in (row.get('event') or {})]
    picks = [t for t, command in events if command.get('action') == 'track']
    # Settling: after the first track command and after each found again
    # from above the route (a stop, a hold, a track); a target picked again
    # in passing (a new id) is scored on.
    anchors = [t for i, (t, command) in enumerate(events)
               if command.get('action') == 'track'
               and (t == picks[0] or (i and events[i - 1][1].get('action') == 'stop'))]
    anchors = anchors or [rows[0]['sim']]
    tracking = sum(state_of(row) == 'tracking' for row in rows if row['sim'] >= anchors[0])
    settled = [row for row in rows
               if state_of(row) == 'tracking'
               and row['sim'] - max([t for t in anchors if t <= row['sim']] or
                                    [float('inf')]) >= settle_s]
    result = {'rows': len(settled), 'mode': step['mode'], 'target': step['target'],
              'seconds': round(rows[-1]['sim'] - anchors[0], 1),
              'reacquisitions': max(0, len(picks) - 1)}
    errors = {key: [] for key in ('horizontal_truth', 'horizontal_nav', 'estimate',
                                  'range_truth', 'depression_truth', 'range_nav',
                                  'depression_nav', 'bearing_nav', 'height_agl_truth')}
    for row in settled:
        target = row_target(row, step['target'])
        if target is None:
            continue
        if row.get('estimate') is not None:
            errors['estimate'].append(math.dist(row['estimate'][:2], target[:2]))
        for frame, drone in (('truth', row.get('truth')), ('nav', row.get('nav'))):
            if drone is None:
                continue
            if step['mode'] == 'centered':
                errors[f'horizontal_{frame}'].append(math.dist(drone[:2], target[:2]))
                if frame == 'truth':
                    errors['height_agl_truth'].append(drone[2] - target[2] - step['altitude'])
            else:
                slant, depression, bearing = slant_geometry(drone, target)
                errors[f'range_{frame}'].append(slant - step['slant_range'])
                errors[f'depression_{frame}'].append(depression - step['depression'])
                if frame == 'nav':
                    # The aircraft sits on the target's bearing from it: the
                    # error is the bearing of the target against the yaw's
                    # image long axis, scored as the stand-off direction.
                    if row.get('yaw') is not None and step.get('axis_deg') is not None:
                        axis = math.degrees(row['yaw']) + step['axis_deg']
                        error = wrap_deg(bearing - axis)
                        errors['bearing_nav'].append(min(abs(error), 180.0 - abs(error)))
    for key, values in errors.items():
        if values:
            if key.startswith(('range', 'depression', 'height')):
                result[key] = stats([abs(v) for v in values])
                result[key + '_mean'] = round(sum(values) / len(values), 3)
            else:
                result[key] = stats(values)
    if errors['depression_truth']:
        result['depression_shallow_truth'] = stats(
            [max(0.0, -value) for value in errors['depression_truth']])
    result['tracking_fraction'] = round(
        tracking / max(1, sum(row['sim'] >= anchors[0] for row in rows)), 3)
    result['reacquire_s'] = [round(t, 2) for t in reacquisitions(rows, anchors[0])]
    result['detector'] = score_ids(dets, step['target'], anchors[0], rows[-1]['sim'])
    failures = []
    if result['tracking_fraction'] < limits['tracking_fraction']:
        failures.append('tracking_fraction')
    if step['mode'] == 'centered':
        checks = (('horizontal_truth', 'median', limits['centered_median_frac'] * step['altitude']),
                  ('horizontal_truth', 'p95', limits['centered_p95_frac'] * step['altitude']))
    else:
        checks = (('range_truth', 'median', limits['range_median_m']),
                  ('range_truth', 'p95', limits['range_p95_m']),
                  ('depression_truth', 'median', limits['depression_median_deg']),
                  ('depression_shallow_truth', 'p95', limits['depression_shallow_p95_deg']))
    for key, statistic, limit in checks:
        if result.get(key) is None or result[key][statistic] > limit:
            failures.append(f'{key} {statistic}')
    detector = result['detector']
    if detector['recall'] is not None and detector['recall'] < limits['recall']:
        failures.append('recall')
    if result['reacquisitions'] > limits['max_reselections']:
        failures.append('reacquisitions')
    if detector.get('following') is not None and detector['following'] < limits['following']:
        failures.append('following')
    result['failures'] = failures
    result['passed'] = not failures
    return result


def score_lost(rows, step, lost_timeout_s, dets=(), limits=THRESHOLDS):
    """The hidden target: lost within lost_timeout_s (+ lost_margin_s),
    hovering (the aircraft moves under hover_drift_m over the last 2 s
    hidden), tracking again within reacquire_s of being shown (under its
    old id, or the new one the test picked)."""
    hidden = [row for row in rows if row.get('step') == step['name'] and row.get('hidden')]
    shown = [row for row in rows if row.get('step') == step['name'] and not row.get('hidden')
             and hidden and row['sim'] > hidden[-1]['sim']]
    result = {'hidden_rows': len(hidden)}
    if not hidden:
        return dict(result, passed=False)
    start = hidden[0]['sim']
    lost_at = next((row['sim'] for row in hidden if state_of(row) == 'lost'), None)
    result['lost_after_s'] = None if lost_at is None else round(lost_at - start, 2)
    tail = [row['nav'] for row in hidden if row['sim'] >= hidden[-1]['sim'] - 2.0 and row.get('nav')]
    result['hover_drift_m'] = round(math.dist(tail[0][:2], tail[-1][:2]), 3) if tail else None
    back = next((row['sim'] for row in shown if state_of(row) == 'tracking'), None)
    result['tracking_again_after_s'] = None if back is None else round(back - hidden[-1]['sim'], 2)
    name = step.get('target')
    seen = next((det['sim'] for det in dets if det['sim'] > hidden[-1]['sim']
                 and (det.get('match') or {}).get(name)), None)
    result['redetected_after_s'] = None if seen is None else round(seen - hidden[-1]['sim'], 2)
    picks = [row['event'] for row in shown if 'reacquire' in (row.get('event') or {})]
    result['new_id'] = bool(picks)
    result['passed'] = bool(
        result['lost_after_s'] is not None
        and result['lost_after_s'] <= lost_timeout_s + limits['lost_margin_s']
        and result['hover_drift_m'] is not None
        and result['hover_drift_m'] < limits['hover_drift_m']
        and result['tracking_again_after_s'] is not None
        and result['tracking_again_after_s'] <= limits['reacquire_s'])
    return result


def score_stop(rows, step, limits=THRESHOLDS):
    """After stop: idle at once, hovering (drift under hover_drift_m over
    the last 5 s of the step)."""
    rows = [row for row in rows if row.get('step') == step['name']]
    if not rows:
        return {'rows': 0, 'passed': False}
    idle_at = next((row['sim'] for row in rows if state_of(row) == 'idle'), None)
    tail = [row['nav'] for row in rows if row['sim'] >= rows[-1]['sim'] - 5.0 and row.get('nav')]
    drift = math.dist(tail[0][:2], tail[-1][:2]) if tail else None
    result = {'idle_after_s': None if idle_at is None else round(idle_at - rows[0]['sim'], 2),
              'hover_drift_m': None if drift is None else round(drift, 3),
              'message': (rows[-1].get('status') or {}).get('message')}
    result['passed'] = bool(idle_at is not None and result['idle_after_s'] <= limits['idle_s']
                            and drift is not None and drift < limits['hover_drift_m'])
    return result


def score_preempt(rows, step):
    rows = [row for row in rows if row.get('step') == step['name']]
    events = [row for row in rows if row.get('event')]
    result = {'events': [row['event'] for row in events]}
    if step['name'] == 'preempt_goto':
        # The GoTo running when the track command came ended without
        # success, and the tracker then followed.
        sent = next((i for i, row in enumerate(events) if 'command' in row['event']), None)
        ended = None if sent is None else next(
            (row['event']['goto_result'] for row in events[sent:]
             if 'goto_result' in row['event']), None)
        following = any(state_of(row) == 'tracking' for row in rows)
        result['goto_result'] = ended
        result['passed'] = bool(ended and ended != 'succeeded' and following)
    else:
        # After the step's track command: tracking, then idle, pre-empted by
        # the expected cause.
        cause = step.get('cause') or {'preempt_map_goal': 'a map goal',
                                      'preempt_goto_behavior': 'GoToBehavior'}[step['name']]
        start = next((row['sim'] for row in events if 'command' in row['event']), None)
        after = [row for row in rows if start is not None and row['sim'] >= start]
        tracked = next((row['sim'] for row in after if state_of(row) == 'tracking'), None)
        idle = [row for row in after if tracked is not None and row['sim'] > tracked
                and state_of(row) == 'idle'
                and cause in ((row.get('status') or {}).get('message') or '')]
        result['message'] = (idle[0]['status']['message'] if idle else None)
        result['idle_after_s'] = round(idle[0]['sim'] - start, 2) if idle else None
        result['passed'] = bool(idle)
    return result


def window_rates(times, window_s=10.0):
    """Frames per second in consecutive windows of window_s over times."""
    times = sorted(times)
    if len(times) < 2:
        return []
    rates, start, count = [], times[0], 0
    for t in times:
        while t >= start + window_s:
            rates.append(count / window_s)
            start += window_s
            count = 0
        count += 1
    return rates


def score_detector(dets, limits=THRESHOLDS, start=None, end=None):
    """The detection frames' rate between start (take-off) and end: per
    10 s of simulation and of wall time (median, 5th percentile)."""
    dets = [det for det in dets if (start is None or det['sim'] >= start)
            and (end is None or det['sim'] <= end)]
    sim = window_rates([det['sim'] for det in dets])
    wall = window_rates([det['rx'] for det in dets])
    result = {'frames': len(dets)}
    for key, rates in (('rate_sim_hz', sim), ('rate_wall_hz', wall)):
        if rates:
            result[key] = {'median': round(percentile(rates, 0.5), 2),
                           'p05': round(percentile(rates, 0.05), 2)}
    result['passed'] = bool(sim and percentile(sim, 0.5) >= limits['detector_rate_hz'])
    return result


def score_navigation(rows):
    """Per step: the navigation's horizontal error against the truth, and
    DemNav's depth frames, matches and fixes and WildNav's fixes per
    minute (the rows' running counts)."""
    result = {}
    for name in dict.fromkeys(row.get('step') for row in rows):
        part = [row for row in rows if row.get('step') == name]
        errors = [math.dist(row['nav'][:2], row['truth'][:2]) for row in part
                  if row.get('nav') and row.get('truth')]
        item = {'error_m': stats(errors)}
        counted = [row for row in part if row.get('navigation')]
        minutes = (counted[-1]['sim'] - counted[0]['sim']) / 60.0 if len(counted) > 1 else 0.0
        if minutes > 0.0:
            for key in counted[0]['navigation']:
                item[f'{key}_per_min'] = round(
                    (counted[-1]['navigation'][key] - counted[0]['navigation'][key]) / minutes, 1)
        result[name] = item
    return result


def rotate(quaternion, vector):
    """vector rotated by an (x, y, z, w) quaternion (world <- body)."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # v + 2 w (q x v) + 2 q x (q x v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ddx, ddy, ddz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ddx), vy + 2 * (w * cy + ddy), vz + 2 * (w * cz + ddz))


def bore_ground(drone, quaternion, ground_up):
    """Where the body's -z axis from drone meets the level ground_up, or
    None (horizontal East, North of the bore's ground point)."""
    direction = rotate(quaternion, (0.0, 0.0, -1.0))
    if direction[2] >= -1e-6:
        return None
    t = (drone[2] - ground_up) / -direction[2]
    return drone[0] + t * direction[0], drone[1] + t * direction[1]


def score_fire(rows, step, limits=THRESHOLDS):
    """One fire command of the fire scenario: the gates when it was sent
    (the fire control's status), the outcome, the shots the tracker and the
    stand-in recorded and the launcher's joints. Expected 'fired' (the car):
    accepted with every gate holding, one shot each, the stand-in's within
    fire_shot_s, the bore's true ground point then within fire_offset_m of
    the car's centre and inside its footprint. Expected 'refused': refused
    with the step's reason, no shot, the rack and trigger never off rest."""
    part = [row for row in rows if row.get('step') == step['name']]
    events = [row for row in part if row.get('event')]
    sent = next((row for row in events
                 if (row['event'].get('command') or {}).get('action') == 'fire'), None)
    outcome = next((row['event'] for row in events if 'fired' in row['event']), None)
    result = {'expect': step['expect'], 'target': step['target']}
    if sent is None or outcome is None:
        return dict(result, passed=False, failures=['no fire command or outcome'])
    fire = sent['event']
    last = outcome.get('last_fire') or {}
    shots, sim_shots = outcome.get('shots') or [], outcome.get('sim_shots') or []
    result.update({key: fire.get(key) for key in ('gates_held', 'fire_block', 'fire_checks',
                                                   'agl_m', 'bore_offset_m')})
    result.update(result_=last.get('result'), reason=last.get('reason'), shots=len(shots),
                  sim_shots=len(sim_shots), launcher_max=outcome.get('launcher_max'))
    result['result'] = result.pop('result_')
    failures = []
    if step['expect'] == 'fired':
        if not fire.get('gates_held') or fire.get('fire_block') != '':
            failures.append('gates')
        if result['result'] != 'fired':
            failures.append('result')
        if len(shots) != 1 or len(sim_shots) != 1:
            failures.append('shots')
        if sim_shots:
            shot = sim_shots[0]
            result['shot_after_s'] = round(shot['sim'] - sent['sim'], 2)
            if result['shot_after_s'] > limits['fire_shot_s']:
                failures.append('shot_after_s')
            target = next((item for item in shot.get('targets') or []
                           if item.get('name') == step['target']), None)
            truth, quaternion = shot.get('truth'), shot.get('quaternion')
            if target is None or not truth or truth[2] is None or not quaternion:
                failures.append('shot truth')
            else:
                point = bore_ground(truth, quaternion, target['up'])
                de, dn = point[0] - target['east'], point[1] - target['north']
                yaw = target.get('yaw', 0.0)
                along = de * math.cos(yaw) + dn * math.sin(yaw)
                across = -de * math.sin(yaw) + dn * math.cos(yaw)
                result.update(shot_offset_truth_m=round(math.hypot(de, dn), 3),
                              shot_along_m=round(along, 3), shot_across_m=round(across, 3),
                              shot_height_truth_m=round(truth[2] - target['up'], 2),
                              shot_tilt_deg=shot.get('att'))
                result['shot_on_car'] = (abs(along) <= CAR_FOOTPRINT_M[0] / 2
                                         and abs(across) <= CAR_FOOTPRINT_M[1] / 2)
                if result['shot_offset_truth_m'] > limits['fire_offset_m']:
                    failures.append('shot_offset_truth_m')
                if not result['shot_on_car']:
                    failures.append('shot_on_car')
    else:
        if result['result'] != 'refused' or step.get('reason', '') not in str(result['reason']):
            failures.append('refusal')
        if shots or sim_shots:
            failures.append('shots')
        rack, trigger = outcome.get('launcher_max') or (None, None)
        if rack is None or rack > 0.002 or trigger > 0.05:
            failures.append('launcher moved')
        if step['name'] == 'fire_person_in_view':
            # Only the person in view refused it: every other check held.
            checks = fire.get('fire_checks') or {}
            if not fire.get('gates_held') or not checks or any(
                    value is (name == 'no_person') for name, value in checks.items()):
                failures.append('other gates')
    result['failures'] = failures
    result['passed'] = not failures
    return result


def score(rows, steps, settle_s=20.0, lost_timeout_s=3.0, dets=(), limits=THRESHOLDS):
    report = {}
    for step in steps:
        if step['kind'] == 'follow':
            report[step['name']] = score_follow(rows, step, settle_s, dets, limits)
        elif step['kind'] == 'lost':
            report[step['name']] = score_lost(rows, step, lost_timeout_s, dets, limits)
        elif step['kind'] == 'stop':
            report[step['name']] = score_stop(rows, step, limits)
        elif step['kind'] == 'preempt':
            report[step['name']] = score_preempt(rows, step)
        elif step['kind'] == 'fire':
            report[step['name']] = score_fire(rows, step, limits)
    if dets:
        flying = next((row['sim'] for row in rows if (row.get('event') or {}).get('takeoff')),
                      None)
        report['detector'] = score_detector(dets, limits, flying,
                                            rows[-1]['sim'] if rows else None)
    if rows:
        report['navigation'] = score_navigation(rows)
    return report


def read_recording(path):
    rows, steps, dets, meta = [], [], [], {}
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            item = json.loads(line)
            kind = item.get('k')
            if kind == 'step':
                steps.append(item['step'])
            elif kind == 'row':
                rows.append(item)
            elif kind == 'det':
                dets.append(item)
            elif kind == 'meta':
                meta = item
    return rows, steps, dets, meta


# --- resources ----------------------------------------------------------------
DETECTOR_LINE = re.compile(r'tracking_detector[^|]*\|.*?([\d.]+) frames/s, (\d+) ms each, '
                           r'(\d+) tracks')


def score_resources(cpu_lines=(), log_lines=()):
    """CPU while the test ran (cpu_lines: tracking_sim_test.sh's samples,
    JSON {'t', 'host_busy_pct', 'cores', 'containers': {name: percent of
    one core}}) and the detector's own report (stack log lines: frames/s
    and ms per frame over each 10 s of wall time)."""
    result = {}
    samples = [json.loads(line) for line in cpu_lines if line.strip()]
    if samples:
        result['host_busy_pct'] = stats([s['host_busy_pct'] for s in samples])
        result['cores'] = samples[-1].get('cores')
        containers = {}
        for sample in samples:
            for name, value in (sample.get('containers') or {}).items():
                containers.setdefault(name, []).append(value)
        result['containers_pct_of_core'] = {
            name: {'median': round(percentile(values, 0.5), 1),
                   'p95': round(percentile(values, 0.95), 1)}
            for name, values in sorted(containers.items(),
                                       key=lambda item: -percentile(item[1], 0.5))}
    reports = [match.groups() for match in map(DETECTOR_LINE.search, log_lines) if match]
    if reports:
        result['detector_wall_hz'] = stats([float(rate) for rate, _, _ in reports])
        result['detector_ms_per_frame'] = stats([float(ms) for _, ms, _ in reports
                                                 if float(ms) > 0])
    return result


def host_busy(previous, current):
    """Busy percentage of all cores between two /proc/stat 'cpu' lines."""
    a = [int(value) for value in previous.split()[1:]]
    b = [int(value) for value in current.split()[1:]]
    delta = [y - x for x, y in zip(a, b)]
    total = sum(delta)
    idle = delta[3] + (delta[4] if len(delta) > 4 else 0)  # idle + iowait
    return 100.0 * (total - idle) / total if total > 0 else 0.0


def sample_cpu(args):
    """Every --interval-s until stopped: the host's busy percentage and each
    of the compose project's containers' CPU (docker stats, percent of one
    core), one JSON line each."""
    import os
    import subprocess

    def stat_line():
        with open('/proc/stat', encoding='ascii') as stream:
            return stream.readline()

    previous = stat_line()
    with open(args.output, 'a', encoding='utf-8') as output:
        while True:
            started = time.time()
            text = subprocess.run(
                ['docker', 'stats', '--no-stream', '--format', '{{.Name}} {{.CPUPerc}}'],
                capture_output=True, text=True, check=False).stdout
            containers = {}
            for line in text.splitlines():
                name, _, value = line.partition(' ')
                if name.startswith(args.project + '-'):
                    service = name[len(args.project) + 1:].rsplit('-', 1)[0]
                    containers[service] = float(value.strip().rstrip('%') or 0.0)
            current = stat_line()
            output.write(json.dumps({'t': round(started, 1),
                                     'host_busy_pct': round(host_busy(previous, current), 1),
                                     'cores': os.cpu_count(), 'containers': containers}) + '\n')
            output.flush()
            previous = current
            time.sleep(max(0.0, args.interval_s - (time.time() - started)))


# --- flying -------------------------------------------------------------------
def fly(args):
    import rclpy
    from as2_msgs.action import GoToWaypoint, Land, Takeoff
    from as2_msgs.msg import PlatformInfo, YawMode
    from geographic_msgs.msg import GeoPointStamped
    from geometry_msgs.msg import PointStamped, PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, NavSatFix
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool
    from vision_msgs.msg import Detection2DArray

    rclpy.init()
    node = rclpy.create_node('tracking_sim_test')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    log = node.get_logger()
    ns = '/' + args.namespace.strip('/')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'sim': None, 'origin': None, 'nav': None, 'yaw': None, 'truth_world': None,
             'attitude': None,
             'targets': None, 'status': None, 'estimate': None, 'platform': None,
             'step': 'setup', 'hidden': '', 'offset': [], 'up_offset': None,
             'cmd': None, 'frames': [], 'halted': '', 'shots': [], 'sim_shots': [],
             'launcher': None, 'launcher_max': [0.0, 0.0], 'quaternion': None,
             'navigation': {'demnav_frames': 0, 'demnav_valid': 0, 'demnav_fixes': 0,
                            'wildnav_fixes': 0}}
    lock = threading.Lock()
    pairing = FramePairing()
    output = open(args.output, 'w', encoding='utf-8')  # noqa: SIM115

    def sim_now():
        return state['sim'] if state['sim'] is not None else time.time()

    def write(item):
        with lock:
            if not output.closed:
                output.write(json.dumps(item, separators=(',', ':')) + '\n')
                output.flush()

    def on_clock(msg):
        state['sim'] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_origin(msg):
        state['origin'] = (msg.position.latitude, msg.position.longitude, msg.position.altitude)

    def on_odometry(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        state['nav'] = (p.x, p.y, p.z)
        state['yaw'] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_truth(msg):
        pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
        state['truth_world'] = (pose.position.x, pose.position.y, pose.position.z)
        q = pose.orientation
        state['attitude'] = tilt_deg((q.x, q.y, q.z, q.w))
        state['quaternion'] = (q.x, q.y, q.z, q.w)

    tracks = {}  # name: [(sim, east, north, up)], the last 120 s

    def on_targets(msg):
        try:
            state['targets'] = json.loads(msg.data)['targets']
        except (ValueError, KeyError):
            return
        if state['sim'] is None:
            return  # before /clock: sim_now() would be wall time and stick the history
        now = sim_now()
        for item in state['targets']:
            track = tracks.setdefault(item['name'], [])
            if track and now < track[-1][0]:
                track.clear()  # the simulation went back
            if not track or now - track[-1][0] >= 0.5:
                track.append((now, item['east'], item['north'], item['up']))
                while track and now - track[0][0] > 120.0:
                    track.pop(0)

    def on_status(msg):
        try:
            state['status'] = json.loads(msg.data)
        except ValueError:
            pass

    def on_estimate(msg):
        state['estimate'] = (msg.point.x, msg.point.y, msg.point.z)

    def boxes(msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        return stamp, [{'id': item.id, 'cx': item.bbox.center.position.x,
                        'cy': item.bbox.center.position.y,
                        'w': item.bbox.size_x, 'h': item.bbox.size_y,
                        'cls': item.results[0].hypothesis.class_id if item.results else ''}
                       for item in msg.detections]

    def frame(stamp, detections, truth):
        """A detection frame paired with its truth boxes: which track is
        on each target in view."""
        names = {item['name'] for item in state['targets'] or []}
        visible = sorted(box['id'] for box in truth if box['id'] in names)
        match = associate([box for box in truth if box['id'] in names], detections)
        with lock:
            state['frames'] = state['frames'][-9:] + [match]
        if state['sim'] is None:
            return  # before /clock: no simulation time to file it under
        write({'k': 'det', 'rx': round(time.time(), 3), 'sim': round(sim_now(), 3),
               'stamp': round(stamp * 1e-9, 3), 'step': state['step'],
               'n': len(detections), 'visible': visible, 'match': match,
               'fp': len(detections) - len(match), 'cmd': following()})

    def on_detections(msg):
        stamp, detections = boxes(msg)
        if args.detector == 'truth':
            # The detections are the truth boxes, named after the targets.
            frame(stamp, detections, detections)
            return
        with lock:
            pairing.add_detections(stamp, detections)
            paired = pairing.ready()
        for item in paired:
            frame(*item)

    def on_truth_boxes(msg):
        stamp, truth = boxes(msg)
        with lock:
            pairing.add_truth(stamp, truth)
            paired = pairing.ready()
        for item in paired:
            frame(*item)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(GeoPointStamped, '/navigation/origin', on_origin, latched)
    node.create_subscription(Odometry, '/navigation/odometry', on_odometry, qos_profile_sensor_data)
    truth_topic, truth_type = TRUTH[args.stack]
    node.create_subscription(PoseStamped if truth_type == 'pose' else Odometry, truth_topic,
                             on_truth, qos_profile_sensor_data)
    node.create_subscription(String, args.targets_topic, on_targets, 10)
    node.create_subscription(String, '/tracking/status', on_status, latched)
    node.create_subscription(PointStamped, '/tracking/target', on_estimate, 10)
    node.create_subscription(Detection2DArray, '/tracking/detections', on_detections,
                             qos_profile_sensor_data)
    if args.detector == 'real':
        node.create_subscription(Detection2DArray, args.truth_boxes_topic, on_truth_boxes,
                                 qos_profile_sensor_data)
    node.create_subscription(PlatformInfo, f'{ns}/platform/info',
                             lambda msg: state.__setitem__('platform', msg), 10)

    # The foam dart launcher: the tracker's shots, the stand-in's (its sear
    # released) with the aircraft's and the targets' truth then, and its
    # joints (the most the rack and trigger moved in a step).
    def on_shot(msg, key):
        try:
            item = json.loads(msg.data)
        except ValueError:
            return
        shot = {'sim': round(sim_now(), 3), 'shot': item, 'truth': truth_nav(),
                'quaternion': state['quaternion'], 'targets': state['targets'],
                'nav': state['nav'], 'att': state['attitude']}
        state[key].append(shot)
        record({key: shot})

    def on_launcher_joints(msg):
        names = list(msg.name)
        values = dict(zip(names, msg.position))
        if 'actuator_joint' in values and 'trigger_joint' in values:
            state['launcher'] = (values['actuator_joint'], values['trigger_joint'])
            state['launcher_max'] = [max(state['launcher_max'][0], values['actuator_joint']),
                                     max(state['launcher_max'][1], values['trigger_joint'])]

    node.create_subscription(String, '/tracking/shots', lambda msg: on_shot(msg, 'shots'), 10)
    node.create_subscription(String, '/simulation/launcher/shots',
                             lambda msg: on_shot(msg, 'sim_shots'), 10)
    node.create_subscription(JointState, '/aerodrone/launcher/joint_states', on_launcher_joints,
                             qos_profile_sensor_data)
    counts = state['navigation']

    def count(*keys):
        for key in keys:
            counts[key] += 1

    # DemNav's matching (a /demnav/valid per depth frame, true when it
    # matched) and the fixes the navigation fuses.
    node.create_subscription(
        Bool, '/demnav/valid',
        lambda msg: count('demnav_frames', *(['demnav_valid'] if msg.data else [])), 10)
    node.create_subscription(NavSatFix, '/demnav/fix', lambda msg: count('demnav_fixes'), 10)
    node.create_subscription(NavSatFix, '/wildnav/fix', lambda msg: count('wildnav_fixes'), 10)
    command_pub = node.create_publisher(String, '/tracking/command', 10)
    hide_pubs = [node.create_publisher(String, topic, 10)
                 for topic in ('/simulation/targets/hide', '/tracking/truth/hide')]
    goal_pub = node.create_publisher(String, '/aerostack/map_goal', 10)
    halt_pub = node.create_publisher(String, '/simulation/targets/halt', 10)
    arm = node.create_client(SetBool, f'{ns}/set_arming_state')
    offboard = node.create_client(SetBool, f'{ns}/set_offboard_mode')
    takeoff = ActionClient(node, Takeoff, f'{ns}/TakeoffBehavior')
    goto = ActionClient(node, GoToWaypoint, f'{ns}/GoToBehavior')
    land = ActionClient(node, Land, f'{ns}/LandBehavior')
    threading.Thread(target=executor.spin, daemon=True).start()

    def truth_nav():
        world, origin = state['truth_world'], state['origin']
        if world is None or origin is None:
            return None
        east, north = world_to_navigation(world[0], world[1], args.world_origin, origin)
        up = None if state['up_offset'] is None else world[2] - state['up_offset']
        return (east, north, up)

    def record(event=None):
        truth = truth_nav()
        with lock:
            ids = dict(state['frames'][-1]) if state['frames'] else {}
        row = {'k': 'row', 'rx': round(time.time(), 3), 'sim': round(sim_now(), 3),
               'step': state['step'], 'hidden': state['hidden'],
               'nav': state['nav'], 'yaw': state['yaw'],
               'truth': truth if truth and truth[2] is not None else None,
               'att': state['attitude'],
               'targets': state['targets'], 'status': state['status'],
               'estimate': state['estimate'], 'cmd': state['cmd'], 'ids': ids,
               'navigation': dict(state['navigation']), 'halted': state['halted'],
               'launcher': state['launcher']}
        if event is not None:
            row['event'] = event
            log.info(f'{state["step"]}: {event}')
        write(row)

    def wait(seconds, condition=None):
        """Record at 10 Hz for seconds of simulation time, or until
        condition() holds; True when it did."""
        end = sim_now() + seconds
        while sim_now() < end:
            record()
            if condition is not None and condition():
                return True
            time.sleep(0.1)
        return condition is None

    def require(ok, message):
        if not ok:
            record({'error': message})
            raise RuntimeError(message)

    def call(client, value):
        require(client.wait_for_service(timeout_sec=30.0), f'{client.srv_name} unavailable')
        future = client.call_async(SetBool.Request(data=value))
        end = time.time() + 30.0
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        return future.done() and future.result().success

    def send_goal(client, goal, timeout_s=30.0, attempts=10):
        """Send an action goal; a refused one is sent again 1 s later: as2's
        behaviours refuse a goal until a state has arrived since their last
        goal ended ("there is no localization"), slow in a slow simulator."""
        require(client.wait_for_server(timeout_sec=60.0), 'action server unavailable')
        for _ in range(attempts):
            future = client.send_goal_async(goal)
            end = time.time() + timeout_s
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                return handle
            wait(1.0)
        require(False, 'goal refused')

    def result_name(future):
        if not future.done():
            return None
        return {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(future.result().status,
                                                                 str(future.result().status))

    def goto_goal(east, north, up, speed=3.0, yaw=None):
        goal = GoToWaypoint.Goal()
        goal.target_pose.header.frame_id = 'earth'
        goal.target_pose.point.x, goal.target_pose.point.y, goal.target_pose.point.z = (
            float(east), float(north), float(up))
        if yaw is None:
            goal.yaw.mode = YawMode.KEEP_YAW
        else:
            goal.yaw.mode = YawMode.FIXED_YAW
            goal.yaw.angle = float(yaw)
        goal.max_speed = float(speed)
        return goal

    def command(data, event=None):
        command_pub.publish(String(data=json.dumps(data)))
        if data.get('action') == 'track':
            state['cmd'] = data['target_id']
        record(dict(event or {}, command=data))

    def status_state():
        return (state['status'] or {}).get('state')

    def following():
        """The id the tracker follows: the one sent, or the one it handed
        the target over to."""
        return (state['status'] or {}).get('target_id') or state['cmd']

    def target_truth(name):
        for item in state['targets'] or []:
            if item.get('name') == name:
                return item
        return None

    def current_id(name):
        with lock:
            return stable_id(state['frames'], name)

    def pick(name, timeout_s=30.0):
        """The id of the box on target name, as the operator clicks it:
        the one it has had in the last three detection frames."""
        end = sim_now() + timeout_s
        while sim_now() < end:
            target_id = current_id(name)
            if target_id is not None:
                return target_id
            record()
            time.sleep(0.1)
        return None

    def track(name, fields, event=None):
        """Pick the target and send the track command with its id."""
        target_id = pick(name)
        require(target_id is not None, f'{name} not detected')
        command(dict({'action': 'track', 'target_id': target_id}, **fields),
                dict(event or {}, target=name))
        return target_id

    def begin(step):
        state['step'] = step['name']
        write({'k': 'step', 'step': step})
        log.info(f'step {step["name"]}')

    def hide(name):
        for publisher in hide_pubs:
            publisher.publish(String(data=name))

    try:
        # --- ready -----------------------------------------------------------
        log.info('waiting for the navigation origin, odometry, truth, targets, '
                 'detections and tracker')
        end = time.time() + args.ready_timeout_s

        def ready():
            return {'origin': state['origin'], 'nav': state['nav'],
                    'truth_world': state['truth_world'], 'targets': state['targets'],
                    'status': state['status'], 'platform': state['platform'],
                    'detections': state['frames']}

        while time.time() < end and not all(ready().values()):
            time.sleep(0.5)
        require(all(value for key, value in ready().items() if key != 'platform'),
                f'not ready: { {key: bool(value) for key, value in ready().items()} }')
        # The navigation frame's up is the world's minus the home's: the
        # aircraft sits at home until take-off.
        samples = []
        for _ in range(20):
            if state['truth_world'] and state['nav']:
                samples.append(state['truth_world'][2] - state['nav'][2])
            time.sleep(0.1)
        state['up_offset'] = sorted(samples)[len(samples) // 2]
        write({'k': 'meta', 'stack': args.stack, 'detector': args.detector,
               'origin': state['origin'], 'world_origin': args.world_origin,
               'up_offset': state['up_offset'], 'args': vars(args)})
        # --- take off --------------------------------------------------------
        state['step'] = 'takeoff'
        require(call(offboard, True), 'offboard refused')
        require(call(arm, True), 'arming refused')
        goal = Takeoff.Goal(takeoff_height=float(args.takeoff_height), takeoff_speed=2.0)
        handle = send_goal(takeoff, goal)
        future = handle.get_result_async()
        wait(240.0, future.done)
        require(result_name(future) == 'succeeded', f'take-off {result_name(future)}')
        record({'takeoff': 'done'})
        wait(3.0)

        def near(name, radius):
            item, nav = target_truth(name), state['nav']
            return bool(item and nav and math.dist(nav[:2], (item['east'], item['north'])) < radius)

        def hold_over(name, altitude=None):
            """Fly altitude (--altitude) above the target's route where it
            comes nearest the route's middle (route_hold: from its last 60 s
            of truth) and wait for it to pass below."""
            altitude = args.altitude if altitude is None else altitude
            now = sim_now()
            samples = [sample[1:] for sample in tracks.get(name, []) if now - sample[0] <= 60.0]
            require(samples, f'no truth for {name}')
            east, north, up = route_hold(samples)
            handle = send_goal(goto, goto_goal(east, north, up + altitude, 4.0))
            result = handle.get_result_async()
            wait(240.0, result.done)
            record({'hold': [east, north, up + altitude], 'goto_result': result_name(result)})
            require(wait(180.0, lambda: near(name, 16.0)), f'{name} never passed below')

        def follow(name, fields, seconds):
            """Track name for seconds of simulation time. Lost under a new
            id (the detector lost it for longer than its max_lost_s), it is
            tracked under that id --repick-s later; out of view for 10 s
            (e.g. after a navigation glitch), it is found again from above
            its route and tracked anew."""
            end = sim_now() + seconds
            track(name, fields)
            lost_since = None
            while sim_now() < end:
                record()
                time.sleep(0.1)
                # Simulation seconds, as the tracker's lost_timeout_s.
                if status_state() not in ('lost', 'error'):
                    lost_since = None
                    continue
                lost_since = sim_now() if lost_since is None else lost_since
                lost_for = sim_now() - lost_since
                if lost_for < args.repick_s:
                    continue
                target_id = current_id(name)
                if target_id is not None and target_id != following():
                    command(dict({'action': 'track', 'target_id': target_id}, **fields),
                            {'reacquire': name, 'lost_s': round(lost_for, 1)})
                    lost_since = None
                elif lost_for >= 10.0:
                    record({'reacquire': name, 'lost_s': round(lost_for, 1)})
                    command({'action': 'stop'})
                    hold_over(name)
                    track(name, fields)
                    lost_since = None

        def launcher_status():
            return state['status'] or {}

        def wait_gates(seconds, ok):
            """Record until ok(status) holds (True) or seconds pass; the
            fire_block reasons seen meanwhile are recorded with the result."""
            seen = {}

            def check():
                status = launcher_status()
                block = status.get('fire_block')
                if block is not None:
                    key = re.sub(r'[\d.]+', '#', str(block))
                    seen[key] = seen.get(key, 0) + 1
                return ok(status)

            done = wait(seconds, check)
            record({'gates_waited': round(seconds, 1), 'gates_ok': done,
                    'blocks_seen': dict(sorted(seen.items(), key=lambda item: -item[1])[:8])})
            return done

        def checks_but(status, *names):
            """Every fire check true but those named (which must be false)."""
            checks = status.get('fire_checks') or {}
            return bool(checks) and all(checks.get(name) is (name not in names)
                                        for name in checks)

        def above(name, altitude, yaw=0.0, speed=4.0):
            """GoTo altitude m above the target's truth, nose at yaw (ENU
            rad: 0 east puts the image's long axis north-south)."""
            item = target_truth(name)
            require(item is not None, f'no truth for {name}')
            handle = send_goal(goto, goto_goal(item['east'], item['north'],
                                               item['up'] + altitude, speed, yaw))
            result = handle.get_result_async()
            wait(240.0, result.done)
            record({'above': name, 'altitude': altitude, 'goto_result': result_name(result)})

        def halt_where(name, condition, timeout_s=150.0):
            """Let name go on its route until condition(truth) holds, then
            stop it there (/simulation/targets/halt)."""
            halt_pub.publish(String(data=''))
            require(wait(timeout_s, lambda: condition(target_truth(name) or {})),
                    f'{name} never reached its halt point')
            halt_pub.publish(String(data=name))
            state['halted'] = name
            require(wait(5.0, lambda: (target_truth(name) or {}).get('halted') is True),
                    f'{name} not halted')
            record({'halt': name, 'at': [round(target_truth(name)[key], 2)
                                         for key in ('east', 'north', 'up')]})

        def fire_at(target_name, expect, step_name, altitude, settle_s, gates, reason=None):
            """Track target_name centered at altitude, arm, wait until
            gates(status) holds, send one fire command naming the tracked
            target, and record the outcome."""
            step = {'name': step_name, 'kind': 'fire', 'target': target_name,
                    'expect': expect, 'altitude': altitude, 'reason': reason}
            begin(step)
            command({'action': 'stop'})
            above(target_name, args.fire_approach_altitude)
            require(wait(60.0, lambda: current_id(target_name)), f'{target_name} not detected')
            track(target_name, {'mode': 'centered', 'altitude_agl_m': altitude})
            require(wait(30.0, lambda: status_state() == 'tracking'),
                    f'{target_name} not tracked')
            wait(settle_s)
            command({'action': 'launcher_arm'})
            require(wait(5.0, lambda: launcher_status().get('launcher_armed') is True),
                    'the launcher did not arm')
            held = wait_gates(args.fire_gates_s, gates)
            status = dict(launcher_status())
            target_id = status.get('target_id')
            state['launcher_max'] = [0.0, 0.0]
            shots_before = (len(state['shots']), len(state['sim_shots']))
            command({'action': 'fire', 'target_id': target_id},
                    {'fire': target_name, 'gates_held': held,
                     'fire_block': status.get('fire_block'),
                     'fire_checks': status.get('fire_checks'),
                     'agl_m': status.get('agl_m'), 'bore_offset_m': status.get('bore_offset_m')})
            previous = (status.get('last_fire') or {}).get('stamp')

            def ended():
                last = launcher_status().get('last_fire') or {}
                return (last.get('stamp') != previous
                        and last.get('result') in ('fired', 'fault', 'refused')
                        and launcher_status().get('launcher') not in ('priming', 'chambering',
                                                                       'firing'))

            wait(15.0, ended)
            wait(3.0)  # any shot after it
            record({'fired': target_name,
                    'last_fire': launcher_status().get('last_fire'),
                    'shots': state['shots'][shots_before[0]:],
                    'sim_shots': state['sim_shots'][shots_before[1]:],
                    'launcher_max': state['launcher_max']})
            command({'action': 'launcher_disarm'})
            wait(2.0)

        def fire_scenario():
            """The launcher's fire control (systems/aerodrone/tracking.md,
            "Foam dart launcher") against the stand-in launcher: a shot at
            the halted car (allowed), a fire command at a person (refused),
            and one at the car with a person in view (refused)."""
            command({'action': 'reload', 'darts': 6})
            wait(2.0, lambda: launcher_status().get('darts') == 6)
            require(launcher_status().get('darts') == 6, 'reload refused')
            people = [item['name'] for item in state['targets'] or []
                      if item.get('class') == 'person']

            def clear_of_people(item):
                """The car at least --fire-clear-m from every person."""
                if 'east' not in item:
                    return False
                return all(math.dist((item['east'], item['north']),
                                     (other['east'], other['north'])) >= args.fire_clear_m
                           for other in map(target_truth, people) if other)

            # 1. The car, halted where it is 40 m or more from the people.
            begin({'name': 'fire_setup', 'kind': 'setup'})
            above(args.target, args.fire_approach_altitude + 20.0)
            halt_where(args.target, clear_of_people)
            fire_at(args.target, 'fired', 'fire_car', args.fire_altitude, args.fire_settle_s,
                    lambda status: status.get('fire_block') == '')
            halt_pub.publish(String(data=''))  # the car laps again meanwhile (step 3's route)
            released = sim_now()
            # 2. A person (standing): refused, never at people.
            fire_at(args.fire_person, 'refused', 'fire_person', args.fire_altitude,
                    args.fire_settle_s,
                    lambda status: checks_but(status, 'not_person', 'no_person'),
                    reason='never at people')
            # 3. The car halted next to the walking person's loop, from where
            # the camera sees the person on part of every lap: refused while
            # the person is in view, every other check holding.
            begin({'name': 'fire_setup_view', 'kind': 'setup'})
            command({'action': 'stop'})
            # A whole lap of the car since it went on (77 s), and the walking
            # person's loop (24 s): the point of the car's route nearest the
            # middle of the loop, whose near edge is then about 9.5 m away,
            # in the image from 19 m with the image's long axis across the
            # route.
            wait(max(0.0, released + 85.0 - sim_now()))
            walk = [sample[1:3] for sample in tracks.get(args.fire_view_person, [])
                    if sample[0] >= released]
            route = [sample[1:3] for sample in tracks.get(args.target, [])
                     if sample[0] >= released]
            require(walk and route, f'no truth for {args.fire_view_person} or {args.target}')
            middle = [sum(values) / len(values) for values in zip(*walk)]
            near = min(route, key=lambda point: math.dist(point, middle))
            record({'view_halt': [round(value, 2) for value in near],
                    'person_middle': [round(value, 2) for value in middle]})

            def close(item):
                return math.dist((item.get('east', 1e9), item.get('north', 1e9)), near) < 3.0

            above(args.target, args.fire_approach_altitude + 20.0)
            halt_where(args.target, close)
            fire_at(args.target, 'refused', 'fire_person_in_view',
                    args.fire_view_altitude, args.fire_settle_s,
                    lambda status: checks_but(status, 'no_person')
                    and 'is in view' in str(status.get('fire_block')),
                    reason='a person is in view')
            command({'action': 'stop'})
            halt_pub.publish(String(data=''))
            state['halted'] = ''

        if args.scenario == 'tracking':
            centered = {'mode': 'centered', 'altitude_agl_m': args.altitude}
            # The person of the lost and pre-emption steps, from lower down.
            low = {'mode': 'centered', 'altitude_agl_m': args.lost_altitude}
            # --- 1: a track pre-empts a GoTo ------------------------------------------
            step = {'name': 'preempt_goto', 'kind': 'preempt'}
            begin(step)
            hold_over(args.target)
            require(wait(60.0, lambda: near(args.target, 8.0) and current_id(args.target)),
                    f'{args.target} not detected below')
            nav = state['nav']
            handle = send_goal(goto, goto_goal(nav[0] + 80.0, nav[1], nav[2], 2.0))
            goto_result = handle.get_result_async()
            record({'goto': 'away, 80 m east'})
            # The GoTo runs, slowly enough that the target stays in view.
            wait(1.0)
            target_id = pick(args.target, 5.0)
            require(target_id is not None, f'{args.target} out of view once the GoTo started')
            command(dict({'action': 'track', 'target_id': target_id}, **centered),
                    {'target': args.target})
            wait(10.0, goto_result.done)
            record({'goto_result': result_name(goto_result)})
            wait(10.0, lambda: status_state() == 'tracking')
            wait(2.0)
            # --- 2: centered ------------------------------------------------------------
            step = {'name': 'centered', 'kind': 'follow', 'mode': 'centered',
                    'target': args.target, 'altitude': args.altitude}
            begin(step)
            follow(args.target, centered, args.centered_s)
            # --- 3: slant ---------------------------------------------------------------
            axis = args.mount_yaw_deg - 90.0  # the image's long (horizontal) axis
            step = {'name': 'slant', 'kind': 'follow', 'mode': 'slant', 'target': args.target,
                    'slant_range': args.slant_range, 'depression': args.depression,
                    'axis_deg': axis}
            begin(step)
            follow(args.target, {'mode': 'slant', 'slant_range_m': args.slant_range,
                                 'depression_deg': args.depression}, args.slant_s)
            # --- 4: lost ----------------------------------------------------------------
            step = {'name': 'lost', 'kind': 'lost', 'target': args.lost_target}
            begin(step)
            command({'action': 'stop'})
            hold_over(args.lost_target, args.lost_altitude)
            track(args.lost_target, low)
            require(wait(30.0, lambda: status_state() == 'tracking'),
                    f'{args.lost_target} not tracked')
            wait(15.0)
            state['hidden'] = args.lost_target
            hide(args.lost_target)
            record({'hide': args.lost_target})
            wait(args.hidden_s)
            state['hidden'] = ''
            hide('')
            record({'show': args.lost_target})
            target_id = pick(args.lost_target, 15.0)
            record({'redetected': args.lost_target, 'id': target_id})
            if target_id is not None and target_id != following():
                command(dict({'action': 'track', 'target_id': target_id}, **low),
                        {'reacquire': args.lost_target})
            wait(10.0, lambda: status_state() == 'tracking')
            wait(5.0)
            # --- 5: stop ----------------------------------------------------------------
            step = {'name': 'stop', 'kind': 'stop'}
            begin(step)
            command({'action': 'stop'})
            wait(args.stop_s)
            # --- 6: pre-emption by a map goal and by a GoTo -------------------------------
            step = {'name': 'preempt_map_goal', 'kind': 'preempt', 'cause': 'a map goal'}
            begin(step)
            track(args.lost_target, low)
            wait(20.0, lambda: status_state() == 'tracking')
            wait(5.0)
            nav = state['nav']
            goal_pub.publish(String(data=f'{nav[0]:.2f};{nav[1]:.2f};{nav[2]:.2f}'))
            record({'map_goal': nav})
            wait(5.0, lambda: status_state() == 'idle')
            wait(2.0)
            step = {'name': 'preempt_goto_behavior', 'kind': 'preempt', 'cause': 'GoToBehavior'}
            begin(step)
            track(args.lost_target, low)
            wait(20.0, lambda: status_state() == 'tracking')
            wait(5.0)
            nav = state['nav']
            handle = send_goal(goto, goto_goal(nav[0] + 5.0, nav[1], nav[2], 2.0))
            record({'goto': 'sent'})
            wait(5.0, lambda: status_state() == 'idle')
            wait(10.0, handle.get_result_async().done)
        else:
            fire_scenario()
        # --- land -------------------------------------------------------------------
        state['step'] = 'land'
        handle = send_goal(land, Land.Goal(land_speed=1.0))
        future = handle.get_result_async()
        wait(180.0, future.done)
        record({'land': result_name(future)})
    finally:
        with lock:
            output.close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='action', required=True)
    run = sub.add_parser('fly')
    run.add_argument('--output', required=True)
    run.add_argument('--stack', choices=sorted(TRUTH), default='gazebo')
    run.add_argument('--detector', choices=('real', 'truth'), default='real',
                     help='real: the YOLO detector (profile tracking); truth: the '
                     'ground-truth detector (profile tracking_truth)')
    run.add_argument('--truth-boxes-topic', default='/tracking/truth/detections',
                     help='the truth boxes the real detector\'s tracks are matched to')
    run.add_argument('--namespace', default='aerodrone')
    run.add_argument('--world-origin', type=float, nargs=2, default=(37.9234, -122.5967),
                     help='INITIAL_LAT INITIAL_LON: the simulator world origin')
    run.add_argument('--targets-topic', default='/simulation/targets')
    run.add_argument('--target', default='car_1')
    run.add_argument('--lost-target', default='person_1')
    run.add_argument('--mount-yaw-deg', type=float, default=0.0)
    run.add_argument('--takeoff-height', type=float, default=35.0)
    run.add_argument('--altitude', type=float, default=50.0,
                     help='centered height above the car (from 50 m the car stays in '
                     'view while the aircraft trails it)')
    run.add_argument('--lost-altitude', type=float, default=40.0,
                     help='height above the person of the lost and pre-emption steps')
    run.add_argument('--slant-range', type=float, default=60.0)
    run.add_argument('--depression', type=float, default=70.0)
    run.add_argument('--centered-s', type=float, default=150.0)
    run.add_argument('--slant-s', type=float, default=150.0)
    run.add_argument('--hidden-s', type=float, default=8.0)
    run.add_argument('--stop-s', type=float, default=15.0)
    run.add_argument('--repick-s', type=float, default=1.0,
                     help='lost this long under a new id: track the new id')
    run.add_argument('--ready-timeout-s', type=float, default=900.0)
    run.add_argument('--scenario', choices=('tracking', 'fire'), default='tracking',
                     help='tracking: the follow, lost, stop and pre-emption steps; fire: the '
                     'foam dart launcher\'s fire control against the stand-in launcher')
    run.add_argument('--fire-person', default='person_2',
                     help='the standing person a fire command is refused at')
    run.add_argument('--fire-altitude', type=float, default=17.0,
                     help='centered height of the fire steps (the fire window is 8-20 m, the '
                     'tracker\'s minimum 15 m)')
    run.add_argument('--fire-view-altitude', type=float, default=19.0,
                     help='centered height above the car halted beside the walking person, '
                     'from where the camera sees the person on part of every lap')
    run.add_argument('--fire-approach-altitude', type=float, default=30.0)
    run.add_argument('--fire-clear-m', type=float, default=40.0,
                     help='the car is halted for the allowed shot this far from every person')
    run.add_argument('--fire-view-person', default='person_1',
                     help='the walking person the car is halted beside (the route\'s point '
                     'nearest the middle of its loop) for the person-in-view step')
    run.add_argument('--fire-settle-s', type=float, default=15.0)
    run.add_argument('--fire-gates-s', type=float, default=120.0,
                     help='how long to wait for the gates before the fire command')
    scorer = sub.add_parser('score')
    scorer.add_argument('recording')
    scorer.add_argument('--settle-s', type=float, default=20.0)
    scorer.add_argument('--lost-timeout-s', type=float, default=3.0)
    scorer.add_argument('--cpu', help="tracking_sim_test.sh's CPU samples (JSON lines)")
    scorer.add_argument('--stack-log', help='the stack log, for the detector\'s own timing')
    cpu = sub.add_parser('cpu', help='sample the host and the containers\' CPU until stopped')
    cpu.add_argument('--project', required=True)
    cpu.add_argument('--output', required=True)
    cpu.add_argument('--interval-s', type=float, default=15.0)
    args = parser.parse_args()
    if args.action == 'fly':
        fly(args)
        return 0
    if args.action == 'cpu':
        sample_cpu(args)
        return 0
    rows, steps, dets, meta = read_recording(args.recording)
    report = score(rows, steps, args.settle_s, args.lost_timeout_s, dets)
    cpu_lines, log_lines = [], []
    try:
        if args.cpu:
            with open(args.cpu, encoding='utf-8') as stream:
                cpu_lines = stream.readlines()
        if args.stack_log:
            with open(args.stack_log, encoding='utf-8', errors='replace') as stream:
                log_lines = [line for line in stream if 'frames/s' in line]
    except OSError as error:
        print(f'resources not scored: {error}', file=sys.stderr)
    if cpu_lines or log_lines:
        report['resources'] = score_resources(cpu_lines, log_lines)
    checks = {name: item['passed'] for name, item in report.items() if 'passed' in item}
    report['summary'] = {'stack': meta.get('stack'), 'detector': meta.get('detector'),
                         'failed': sorted(name for name, ok in checks.items() if not ok),
                         'passed': bool(checks) and all(checks.values())}
    print(json.dumps(report, indent=2))
    return 0 if report['summary']['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

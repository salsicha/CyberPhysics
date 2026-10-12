#!/usr/bin/env python3
"""Score an Aerodrone GPS-denied simulation run (tools/gps_denied_sim_test.sh).

Reads, from a run directory, the recording of the ``gps_denial_recorder``
service (recording.jsonl or recording.jsonl.gz) and the events of the
``gps_denier`` service (denial_events.jsonl) of the Gazebo, Genesis and
Isaac compositions' ``denial`` profile, and reports before (cruise on GPS),
during and after the GPS 1 denial:

- horizontal error against simulator ground truth of the fused navigation
  fix (/navigation/fix) and of ArduPilot's own position (MAVROS global
  position), each against the truth at its arrival: median, p95, max, RMS;
- the errors of the DemNav and WildNav fixes (truth at the fix's camera
  stamp) and of the corrections sent to the autopilot, per source;
- covariance consistency: the share of fused fixes, DemNav/WildNav fixes and
  corrections farther than 2 and 3 sigma from the truth (Mahalanobis distance
  over the reported horizontal covariance; a consistent 2D Gaussian puts
  13.5 % beyond 2 sigma and 1.1 % beyond 3 sigma);
- route-keeping: the true track's distance from the leg the mission
  commanded (the straight line from where the vehicle was when a MAVROS
  position setpoint arrived to that setpoint), and, reported but not
  checked, the waypoints reached and the distance flown (a GoTo completes
  within the compositions' 3 m go_to_threshold of ArduPilot's own position;
  at Aerostack2's 0.2 m default a denied EKF held a waypoint 0.3-0.4 m off,
  stalling the mission);
- whether GPS 1 lost its fix, the vehicle stayed armed in GUIDED with no
  failsafe text (the lite test's FAILSAFE_PATTERN) and corrections stayed
  active.

Phases are cut at the denier's events: cruise -> GPS 1 disabled -> restored
-> done; durations are simulation time. The JSON summary goes to --output
(default <run dir>/score.json); the checks use the stack's thresholds
(STACK_THRESHOLDS, set from the measured runs in tools/gps_denied_sim_test.md;
each is also an option). Exit status 0 = pass, 1 = fail, 2 = unusable run.
"""
import argparse
import bisect
import gzip
import itertools
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# The lite test's failsafe texts.
from gps_denied_sitl_scorer import FAILSAFE_PATTERN

# synthetic_world.METERS_PER_DEG_LAT: lat/lon <-> the simulators' local ENU.
METERS_PER_DEG_LAT = 111320.0
GPS_FIX_TYPE_3D = 3
INACTIVE_SOURCES = ('', 'dead_reckoning')
PHASES = ('before', 'during', 'after')
FIX_SOURCES = ('demnav', 'wildnav', 'correction')
# Truth further apart than this (s) is a gap, not something to interpolate.
MAX_TRUTH_GAP_S = 5.0

COMMON_THRESHOLDS = {
    'min_denied_fraction': 0.95,      # of the configured denial, simulation time
    'max_before_median_m': 1.0,       # fused, cruise on GPS: frames and clocks agree
    'max_after_median_m': 2.0,        # autopilot after GPS 1 returns: recovered
    'min_correction_coverage': 0.8,   # of the denial with an active correction source
    'min_fused_rate_hz': 1.0,         # scored fused samples per simulated second
}
# While GPS 1 is denied (150 s at the default settings): fused and autopilot
# error, the true track's distance from the commanded leg (the same limit as
# the error's maximum), and the share of fused fixes beyond 3 sigma (1.1 %
# for a consistent Gaussian; noisy over one window, as the errors stay
# correlated for about a minute; Gazebo's fused sigma lags its drift, 8 %
# beyond 3 sigma in one run). About twice the worst run of the committed
# navigation code (tools/gps_denied_sim_test.md, "Measured"): the control run
# without DemNav/WildNav, the earlier Gazebo and Isaac results (p95 50-170 m,
# 16 m) and the work-in-progress runs measured there all fail.
STACK_THRESHOLDS = {
    'gazebo': {'max_fused_median_m': 20.0, 'max_fused_p95_m': 35.0, 'max_fused_max_m': 45.0,
               'max_autopilot_median_m': 20.0, 'max_autopilot_p95_m': 35.0,
               'max_autopilot_max_m': 45.0, 'max_route_m': 45.0,
               'max_fused_beyond_3sigma': 0.15},
    'isaac': {'max_fused_median_m': 6.0, 'max_fused_p95_m': 12.0, 'max_fused_max_m': 15.0,
              'max_autopilot_median_m': 6.0, 'max_autopilot_p95_m': 12.0,
              'max_autopilot_max_m': 15.0, 'max_route_m': 15.0,
              'max_fused_beyond_3sigma': 0.10},
    'genesis': {'max_fused_median_m': 5.0, 'max_fused_p95_m': 6.0, 'max_fused_max_m': 7.0,
                'max_autopilot_median_m': 5.0, 'max_autopilot_p95_m': 6.0,
                'max_autopilot_max_m': 7.0, 'max_route_m': 7.0,
                'max_fused_beyond_3sigma': 0.10},
}
STACKS = tuple(STACK_THRESHOLDS)


class Unusable(Exception):
    """The run cannot be scored (exit status 2)."""


# ---------------------------------------------------------------------------
# Small numeric helpers (no numpy: the scorer runs on the host)
# ---------------------------------------------------------------------------
def latlon_to_local(lat, lon, origin_lat, origin_lon):
    """(east, north) metres from the origin, as synthetic_world does."""
    east = (lon - origin_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    return east, (lat - origin_lat) * METERS_PER_DEG_LAT


def percentile(ordered, q):
    """Linear-interpolation percentile of sorted values (numpy's default)."""
    position = (len(ordered) - 1) * q / 100.0
    low = math.floor(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def error_stats(errors):
    values = sorted(e for e in errors if e is not None and math.isfinite(e))
    if not values:
        return {'n': 0}
    return {'n': len(values),
            'median_m': round(percentile(values, 50), 2),
            'p95_m': round(percentile(values, 95), 2),
            'max_m': round(values[-1], 2),
            'rms_m': round(math.sqrt(sum(v * v for v in values) / len(values)), 2)}


def mahalanobis(d_east, d_north, cov):
    """Distance in sigmas over the horizontal covariance [ee, en, nn], or
    None when the covariance is unknown or not positive definite."""
    if not cov or len(cov) != 3:
        return None
    ee, en, nn = (float(c) for c in cov)
    det = ee * nn - en * en
    if not (ee > 0.0 and nn > 0.0 and det > 0.0):
        return None
    return math.sqrt(max(0.0, nn * d_east ** 2 - 2.0 * en * d_east * d_north
                         + ee * d_north ** 2) / det)


def consistency(samples):
    """Share of samples beyond 2 and 3 sigma, and the median sigma."""
    distances, sigmas = [], []
    for sample in samples:
        distance = mahalanobis(sample['de'], sample['dn'], sample['cov'])
        if distance is not None:
            distances.append(distance)
            sigmas.append(math.sqrt((sample['cov'][0] + sample['cov'][2]) / 2.0))
    if not distances:
        return {'n': 0}
    return {'n': len(distances),
            'beyond_2sigma': round(sum(d > 2.0 for d in distances) / len(distances), 3),
            'beyond_3sigma': round(sum(d > 3.0 for d in distances) / len(distances), 3),
            'sigma_median_m': round(percentile(sorted(sigmas), 50), 2)}


def segment_distance(point, start, end):
    px, py = point[0] - start[0], point[1] - start[1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    f = 0.0 if length2 <= 1e-9 else min(1.0, max(0.0, (px * dx + py * dy) / length2))
    return math.hypot(px - f * dx, py - f * dy)


class Track:
    """East/north samples against a time base, linearly interpolated."""

    def __init__(self, points):
        points = sorted(points)
        self.t = [p[0] for p in points]
        self.e = [p[1] for p in points]
        self.n = [p[2] for p in points]

    def covers(self, t, tolerance=1.0):
        return bool(self.t) and self.t[0] - tolerance <= t <= self.t[-1] + tolerance

    def at(self, t, tolerance=1.0):
        if not self.covers(t, tolerance):
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return self.e[0], self.n[0]
        if i == len(self.t):
            return self.e[-1], self.n[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > MAX_TRUTH_GAP_S:
            return None
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        return (self.e[i - 1] + f * (self.e[i] - self.e[i - 1]),
                self.n[i - 1] + f * (self.n[i] - self.n[i - 1]))


class SimClock:
    """Wall arrival time -> simulation time, from the recorded /clock."""

    def __init__(self, pairs):
        pairs = sorted(pairs)
        self.rx = [p[0] for p in pairs]
        self.sim = [p[1] for p in pairs]

    def __call__(self, rx):
        if len(self.rx) < 2:
            return rx  # no clock: wall time is the best there is
        i = min(max(bisect.bisect_left(self.rx, rx), 1), len(self.rx) - 1)
        r0, r1, s0, s1 = self.rx[i - 1], self.rx[i], self.sim[i - 1], self.sim[i]
        return s0 if r1 == r0 else s0 + (rx - r0) * (s1 - s0) / (r1 - r0)


def latest_before(rows, rx):
    """The last change row at or before rx (rows sorted by rx), or None."""
    i = bisect.bisect_right([r['rx'] for r in rows], rx)
    return rows[i - 1] if i else None


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def read_jsonl(path):
    opener = gzip.open if path.endswith('.gz') else open
    rows = []
    with opener(path, 'rt') as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue  # a line cut short when the recorder was stopped
            if isinstance(row, dict):
                rows.append(row)
    return rows


def find(run_dir, name):
    for candidate in (name, name + '.gz'):
        path = os.path.join(run_dir, candidate)
        if os.path.exists(path):
            return path
    return None


def load_run(run_dir):
    recording = find(run_dir, 'recording.jsonl')
    events = find(run_dir, 'denial_events.jsonl')
    if recording is None:
        raise Unusable(f'no recording.jsonl in {run_dir}')
    if events is None:
        raise Unusable(f'no denial_events.jsonl in {run_dir}')
    return read_jsonl(recording), read_jsonl(events)


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------
class Run:
    """A recording on one footing: local ENU metres about the simulators'
    origin, phase windows in wall arrival time, durations in simulation
    time."""

    def __init__(self, rows, events):
        self.by_kind = {}
        for row in rows:
            if 'rx' in row or row.get('k') == 'meta':
                self.by_kind.setdefault(row.get('k'), []).append(row)
        for kind_rows in self.by_kind.values():
            kind_rows.sort(key=lambda r: r.get('rx', 0.0))
        meta = (self.by_kind.get('meta') or [{}])[0]
        if 'origin_lat' not in meta:
            raise Unusable('the recording has no meta row with the projection origin')
        self.meta = meta
        self.origin = (float(meta['origin_lat']), float(meta['origin_lon']))
        truth = self.rows('truth')
        if len(truth) < 2:
            raise Unusable('the recording has no simulator ground truth')
        self.truth_rx = Track([(r['rx'], r['e'], r['n']) for r in truth])
        self.truth_stamp = Track([(r['t'], r['e'], r['n']) for r in truth if 't' in r])
        self.sim = SimClock([(r['rx'], r['sim']) for r in self.rows('clock')])
        self.events = {}
        for event in sorted(events, key=lambda e: e.get('rx', 0.0)):
            self.events.setdefault(event.get('event'), event)
        if 'gps_disabled' not in self.events:
            seen = ', '.join(sorted(k for k in self.events if k)) or 'none'
            raise Unusable(f'GPS 1 was never denied (no gps_disabled event; events: {seen})')
        off = self.events['gps_disabled']['rx']
        on = self.events.get('gps_restored', {}).get('rx')
        end = self.events.get('done', {}).get('rx', truth[-1]['rx'])
        cruise = self.events.get('cruise', {}).get('rx', truth[0]['rx'])
        self.windows = {'before': (cruise, off), 'during': (off, on if on is not None else end)}
        if on is not None:
            self.windows['after'] = (on, max(end, on))

    def rows(self, kind):
        return self.by_kind.get(kind, [])

    def in_window(self, kind, window):
        lo, hi = window
        return [r for r in self.rows(kind) if lo <= r['rx'] < hi]

    def sim_span(self, window):
        lo, hi = window
        return self.sim(hi) - self.sim(lo)

    def local(self, row):
        return latlon_to_local(row['lat'], row['lon'], *self.origin)

    def errors(self, kind, window, at_stamp=False):
        """Fix minus truth for each row of kind in the window; truth at the
        header stamp (camera fixes) when it shares the truth's clock, else
        at arrival. Returns (samples, unscored count)."""
        samples, unscored = [], 0
        for row in self.in_window(kind, window):
            if row.get('status', 0) < 0 and kind in FIX_SOURCES:
                continue  # an invalid DemNav/WildNav fix is not a fix
            if not all(math.isfinite(row.get(key, math.nan)) for key in ('lat', 'lon')):
                unscored += 1
                continue
            truth = None
            if at_stamp and 't' in row and self.truth_stamp.covers(row['t']):
                truth = self.truth_stamp.at(row['t'])
            if truth is None:
                truth = self.truth_rx.at(row['rx'])
            if truth is None:
                unscored += 1
                continue
            east, north = self.local(row)
            samples.append({'rx': row['rx'], 'de': east - truth[0], 'dn': north - truth[1],
                            'cov': row.get('cov')})
        return samples, unscored

    def home(self):
        origin = self.rows('origin')
        if origin:
            return self.local(origin[-1])
        first = self.rows('truth')[0]
        return first['e'], first['n']

    def legs(self):
        """(arrival, start, target) of every commanded leg: setpoints are
        local ENU about the EKF origin (home)."""
        home_e, home_n = self.home()
        legs = []
        for row in self.rows('setpoint'):
            target = (home_e + row['x'], home_n + row['y'])
            start = self.truth_rx.at(row['rx'])
            if start is None:
                start = legs[-1][2] if legs else target
            legs.append((row['rx'], start, target))
        return legs

    def route(self, window):
        legs = self.legs()
        starts = [leg[0] for leg in legs]
        truth = self.in_window('truth', window)
        distances, flown, previous = [], 0.0, None
        for row in truth:
            point = (row['e'], row['n'])
            if previous is not None:
                flown += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
            i = bisect.bisect_right(starts, row['rx'])
            if i:
                _, start, target = legs[i - 1]
                distances.append(segment_distance(point, start, target))
        stats = error_stats(distances)
        stats['waypoints_reached'] = sum(window[0] <= s < window[1] for s in starts)
        stats['flown_m'] = round(flown, 1)
        return stats

    def correction_coverage(self, window):
        sources = self.rows('source')
        fused = self.in_window('fused', window)
        if not fused:
            return 0.0
        active = 0
        for row in fused:
            source = latest_before(sources, row['rx'])
            if source is not None and source.get('v') not in INACTIVE_SOURCES:
                active += 1
        return round(active / len(fused), 3)

    def max_correction_gap(self, window):
        times = [self.sim(window[0])] + [self.sim(r['rx'])
                                          for r in self.in_window('correction', window)]
        times.append(self.sim(window[1]))
        return round(max(b - a for a, b in itertools.pairwise(times)), 1)

    def phase(self, window):
        fused, fused_unscored = self.errors('fused', window)
        autopilot, autopilot_unscored = self.errors('autopilot', window)
        result = {
            'duration_sim_s': round(self.sim_span(window), 1),
            'duration_wall_s': round(window[1] - window[0], 1),
            'fused': {**error_stats([math.hypot(s['de'], s['dn']) for s in fused]),
                      'unscored': fused_unscored, 'consistency': consistency(fused)},
            'autopilot': {**error_stats([math.hypot(s['de'], s['dn']) for s in autopilot]),
                          'unscored': autopilot_unscored},
            'fixes': {},
            'route': self.route(window),
            'correction_coverage': self.correction_coverage(window),
            'max_correction_gap_s': self.max_correction_gap(window),
        }
        for source in FIX_SOURCES:
            samples, _ = self.errors(source, window, at_stamp=source != 'correction')
            result['fixes'][source] = {
                **error_stats([math.hypot(s['de'], s['dn']) for s in samples]),
                'consistency': consistency(samples)}
        return result

    def flight(self):
        """Mode, failsafe texts and GPS 1 fix while denied."""
        lo, hi = self.windows['during']
        states = self.rows('state')
        window_states = [s for s in [latest_before(states, lo)] + self.in_window(
            'state', (lo, hi)) if s is not None]
        left = [f"{s['mode']}{'' if s.get('armed') else ' (disarmed)'}"
                for s in window_states if not (s.get('armed') and s.get('mode') == 'GUIDED')]
        texts = [t['text'] for t in self.in_window('text', (lo, hi))
                 if FAILSAFE_PATTERN.search(t.get('text', ''))]
        gps1 = self.rows('gps1')
        window_fix = [g for g in [latest_before(gps1, lo)] + self.in_window(
            'gps1', (lo, hi)) if g is not None]
        restored = latest_before(gps1, self.windows['after'][1]) if 'after' in self.windows \
            else None
        return {
            'states': [{'mode': s['mode'], 'armed': s.get('armed')} for s in window_states],
            'left_guided': left,
            'failsafe_texts': texts,
            'gps1_fix_types': [g['fix_type'] for g in window_fix],
            'gps1_lost': any(g['fix_type'] < GPS_FIX_TYPE_3D for g in window_fix),
            'gps1_fix_type_at_end': restored['fix_type'] if restored else None,
        }


def check_phase(during, prefix, thresholds):
    stats = during[prefix]
    return {f'{prefix}_{name}': stats.get(f'{name}_m', math.inf)
            <= thresholds[f'max_{prefix}_{name}_m'] for name in ('median', 'p95', 'max')}


def score(rows, events, thresholds, stack=''):
    run = Run(rows, events)
    phases = {name: run.phase(window) for name, window in run.windows.items()}
    flight = run.flight()
    started = run.events.get('started', {})
    configured = float(started.get('duration_s', 0.0)) or None
    disabled, restored = run.events['gps_disabled'], run.events.get('gps_restored')
    denied_sim = (restored['sim'] - disabled['sim']
                  if restored and 'sim' in restored and 'sim' in disabled
                  else phases['during']['duration_sim_s'])
    cruise = run.events.get('cruise')
    denial = {
        'configured': {k: started.get(k) for k in
                       ('start_s', 'duration_s', 'recovery_s', 'cruise_height_m')},
        'denied_sim_s': round(denied_sim, 1),
        'denied_wall_s': phases['during']['duration_wall_s'],
        'real_time_factor': round(denied_sim / phases['during']['duration_wall_s'], 2)
        if phases['during']['duration_wall_s'] > 0 else None,
        'start_after_cruise_sim_s': round(disabled['sim'] - cruise['sim'], 1)
        if cruise and 'sim' in cruise and 'sim' in disabled else None,
        'restored': restored is not None,
        'completed': 'done' in run.events,
    }
    during = phases['during']
    t = thresholds
    checks = {
        'gps_denied_and_restored': restored is not None,
        'denied_duration': configured is not None
        and denied_sim >= t['min_denied_fraction'] * configured,
        'gps1_lost': flight['gps1_lost'],
        'stayed_guided': not flight['left_guided'],
        'no_failsafe': not flight['failsafe_texts'],
        'fused_samples': during['fused']['n']
        >= t['min_fused_rate_hz'] * max(denied_sim, 1.0),
        'before_on_gps': phases['before']['fused'].get('median_m', math.inf)
        <= t['max_before_median_m'],
        **check_phase(during, 'fused', t),
        **check_phase(during, 'autopilot', t),
        'fused_consistency': during['fused']['consistency'].get('beyond_3sigma', 1.0)
        <= t['max_fused_beyond_3sigma'],
        'correction_coverage': during['correction_coverage'] >= t['min_correction_coverage'],
        'route_keeping': during['route'].get('max_m', math.inf) <= t['max_route_m'],
        'recovered': 'after' in phases and phases['after']['autopilot'].get(
            'median_m', math.inf) <= t['max_after_median_m'],
    }
    return {'stack': stack or run.meta.get('stack', ''), 'denial': denial, 'phases': phases,
            'flight': flight, 'thresholds': dict(thresholds), 'checks': checks,
            'passed': all(checks.values())}


def thresholds_for(stack):
    if stack not in STACK_THRESHOLDS:
        raise ValueError(f'unknown stack {stack!r}: one of {", ".join(STACKS)}')
    return {**COMMON_THRESHOLDS, **STACK_THRESHOLDS[stack]}


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def _cell(stats, digits=1):
    if not stats or not stats.get('n'):
        return 'none'
    return (f"{stats['median_m']:.{digits}f} / {stats['p95_m']:.{digits}f} / "
            f"{stats['max_m']:.{digits}f}")


def _counted(stats):
    return f"{_cell(stats)} ({stats.get('n', 0)})"


def _shares(stats):
    if not stats.get('n'):
        return 'no covariance'
    return f"{100 * stats['beyond_2sigma']:.1f} / {100 * stats['beyond_3sigma']:.1f} %"


def report(result):
    denial = result['denial']
    lines = [(f"GPS-denied {result['stack']}: GPS 1 denied {denial['denied_sim_s']} s of "
              f"simulation time ({denial['denied_wall_s']} s wall, "
              f"{denial['real_time_factor']}x real time), "
              f"{denial['start_after_cruise_sim_s']} s after reaching cruise")]
    phases = [p for p in PHASES if p in result['phases']]
    rows = [('', [p for p in phases]),
            ('fused error m (median / p95 / max, n)',
             [_counted(result['phases'][p]['fused']) for p in phases]),
            ('autopilot error m', [_counted(result['phases'][p]['autopilot']) for p in phases]),
            ('route distance m', [_cell(result['phases'][p]['route']) for p in phases])]
    for source in FIX_SOURCES:
        rows.append((f'{source} fix error m',
                     [_counted(result['phases'][p]['fixes'][source]) for p in phases]))
    rows.append(('fused beyond 2 / 3 sigma',
                 [_shares(result['phases'][p]['fused']['consistency']) for p in phases]))
    for source in ('demnav', 'wildnav'):
        rows.append((f'{source} beyond 2 / 3 sigma', [
            _shares(result['phases'][p]['fixes'][source]['consistency']) for p in phases]))
    rows.append(('correction coverage / max gap s', [
        f"{result['phases'][p]['correction_coverage']:.2f} / "
        f"{result['phases'][p]['max_correction_gap_s']}" for p in phases]))
    rows.append(('waypoints reached / flown m', [
        f"{result['phases'][p]['route']['waypoints_reached']} / "
        f"{result['phases'][p]['route']['flown_m']:.0f}" for p in phases]))
    width = max(len(r[0]) for r in rows)
    for label, cells in rows:
        lines.append(f'  {label:<{width}}  ' + ''.join(f'{c:<26}' for c in cells).rstrip())
    flight = result['flight']
    lines.append(f"  while denied: modes {[s['mode'] for s in flight['states']]}, "
                 f"GPS 1 fix types {flight['gps1_fix_types']}, "
                 f"failsafe texts {flight['failsafe_texts'] or 'none'}")
    failed = [name for name, ok in result['checks'].items() if not ok]
    lines.append('  failed checks: ' + (', '.join(failed) if failed else 'none'))
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir')
    parser.add_argument('--stack', choices=STACKS,
                        help="thresholds to use (default: the recording's stack)")
    parser.add_argument('--output', help='JSON summary (default <run_dir>/score.json)')
    names = list(COMMON_THRESHOLDS) + list(STACK_THRESHOLDS['gazebo'])
    for name in names:
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=None)
    args = parser.parse_args(argv)
    output = args.output or os.path.join(args.run_dir, 'score.json')
    try:
        rows, events = load_run(args.run_dir)
        stack = args.stack or next(
            (r.get('stack') for r in rows if r.get('k') == 'meta'), None)
        if stack not in STACK_THRESHOLDS:
            raise Unusable(f'unknown stack {stack!r}: pass --stack')
        thresholds = thresholds_for(stack)
        for name in names:
            if getattr(args, name) is not None:
                thresholds[name] = getattr(args, name)
        result = score(rows, events, thresholds, stack)
    except (OSError, Unusable) as exc:
        print(f'GPS-DENIED SIM ERROR: unusable run: {exc}', file=sys.stderr)
        return 2
    with open(output, 'w') as stream:
        json.dump(result, stream, indent=2)
    print(report(result))
    print(f"GPS-DENIED SIM {result['stack'].upper()} "
          + ('PASS' if result['passed'] else 'FAIL') + f' ({output})')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

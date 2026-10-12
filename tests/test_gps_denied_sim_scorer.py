"""Scorer of the Aerodrone GPS-denied simulation test
(tools/gps_denied_sim_scorer.py): synthetic runs with known errors."""
import gzip
import json
import math

import numpy as np
import pytest
from conftest import REPO, load_script

scorer = load_script(REPO / 'tools/gps_denied_sim_scorer.py', 'gps_denied_sim_scorer_review')
lite = load_script(REPO / 'tools/gps_denied_sitl_scorer.py', 'gps_denied_sitl_scorer_sim_review')

LAT0, LON0 = 37.9234, -122.5967
HOME = (-720.0, -260.0)
RX0 = 1_000_000.0
RTF = 0.2                       # Isaac-like: 1 s of simulation per 5 s of wall time
SPEED = 5.0                     # m/s of simulation time, due east
CRUISE, OFF, ON, DONE = 100.0, 160.0, 310.0, 370.0   # simulation time


def rx_of(sim):
    return RX0 + sim / RTF


def latlon(east, north):
    lat = LAT0 + north / scorer.METERS_PER_DEG_LAT
    lon = LON0 + east / (scorer.METERS_PER_DEG_LAT * math.cos(math.radians(LAT0)))
    return lat, lon


def truth_at(sim):
    return HOME[0] + SPEED * max(0.0, sim - 50.0), HOME[1]


def fix(kind, sim, error=(0.0, 0.0), sigma=5.0, stamp=None, status=0):
    east, north = truth_at(sim)
    lat, lon = latlon(east + error[0], north + error[1])
    return {'k': kind, 'rx': rx_of(sim), 't': rx_of(sim) if stamp is None else stamp,
            'lat': lat, 'lon': lon, 'alt': 1000.0, 'status': status,
            'cov': [sigma ** 2, 0.0, sigma ** 2]}


def during_error(sim):
    """Fused/autopilot error while denied: a ramp from 0 to 10 m north."""
    return 10.0 * (sim - OFF) / (ON - OFF)


def make_run(during=during_error, before=0.1, after=0.3, mode_during='GUIDED',
             texts=(), gps1_lost=True, source='demnav+wildnav', off=OFF, on=ON,
             route_offset=0.0, fused_sigma=6.0, demnav_error=(3.0, 4.0)):
    rows = [{'k': 'meta', 'rx': RX0, 'stack': 'isaac', 'origin_lat': LAT0,
             'origin_lon': LON0}]
    lat, lon = latlon(*HOME)
    rows.append({'k': 'origin', 'rx': RX0, 'lat': lat, 'lon': lon, 'alt': 750.0})
    rows.append({'k': 'state', 'rx': RX0, 'mode': 'GUIDED', 'armed': True})
    rows.append({'k': 'gps1', 'rx': RX0, 'fix_type': 6})
    rows.append({'k': 'source', 'rx': rx_of(CRUISE - 20), 'v': source})
    # Legs due east, each ending ahead of where the vehicle is when the next
    # setpoint arrives: it flies them exactly (route_offset: targets south).
    for sim, east in ((40.0, 800.0), (OFF + 50.0, 1500.0), (ON + 20.0, 2000.0)):
        rows.append({'k': 'setpoint', 'rx': rx_of(sim), 'x': east,
                     'y': -route_offset, 'z': 250.0})
    sim = 0.0
    while sim <= DONE + 10:
        east, north = truth_at(sim)
        rows.append({'k': 'truth', 'rx': rx_of(sim), 't': rx_of(sim), 'e': east,
                     'n': north, 'u': 0.0 if sim < 60 else 200.0})
        rows.append({'k': 'clock', 'rx': rx_of(sim), 'sim': sim})
        if sim >= CRUISE - 30:
            if sim < off:
                error = before
            elif sim < on:
                error = during(sim)
            else:
                error = after
            rows.append(fix('fused', sim, (0.0, error), sigma=fused_sigma))
            rows.append(fix('autopilot', sim, (0.0, error), status=-1 if off <= sim < on else 0))
            if round(sim * 10) % 5 == 0:
                rows.append(fix('demnav', sim, demnav_error, sigma=2.5,
                                stamp=rx_of(sim) - 0.5))
                rows.append(fix('correction', sim, (1.0, 0.0), sigma=40.0))
            if round(sim * 10) % 50 == 0:
                rows.append(fix('wildnav', sim, (0.0, 1.0), sigma=2.0))
        sim = round(sim + 0.2, 3)
    if mode_during != 'GUIDED':
        rows.append({'k': 'state', 'rx': rx_of(off + 20), 'mode': mode_during, 'armed': True})
    if gps1_lost:
        rows.append({'k': 'gps1', 'rx': rx_of(off + 1), 'fix_type': 1})
    rows.append({'k': 'gps1', 'rx': rx_of(on + 1), 'fix_type': 6})
    for sim, text in texts:
        rows.append({'k': 'text', 'rx': rx_of(sim), 'text': text})
    events = [
        {'event': 'started', 'rx': RX0, 'start_s': 60.0, 'duration_s': 150.0,
         'recovery_s': 60.0, 'cruise_height_m': 140.0},
        {'event': 'clock', 'rx': RX0, 'sim': 0.0},
        {'event': 'cruise', 'rx': rx_of(CRUISE), 'sim': CRUISE},
        {'event': 'gps_disabled', 'rx': rx_of(off), 'sim': off},
        {'event': 'gps_restored', 'rx': rx_of(on), 'sim': on},
        {'event': 'done', 'rx': rx_of(DONE), 'sim': DONE},
    ]
    return rows, events


def score(rows, events, **overrides):
    return scorer.score(rows, events, {**scorer.thresholds_for('isaac'), **overrides}, 'isaac')


def write_run(tmp_path, rows, events, compress=False):
    text = ''.join(json.dumps(r) + '\n' for r in rows)
    if compress:
        with gzip.open(tmp_path / 'recording.jsonl.gz', 'wt') as stream:
            stream.write(text)
    else:
        (tmp_path / 'recording.jsonl').write_text(text)
    (tmp_path / 'denial_events.jsonl').write_text(
        ''.join(json.dumps(e) + '\n' for e in events))
    return tmp_path


def test_good_run_passes_with_the_known_errors_per_phase():
    result = score(*make_run())
    assert result['passed'], [k for k, ok in result['checks'].items() if not ok]
    during = result['phases']['during']
    ramp = sorted(during_error(OFF + 0.2 * i) for i in range(750))
    assert during['fused']['n'] == 750
    assert during['fused']['median_m'] == pytest.approx(np.percentile(ramp, 50), abs=0.01)
    assert during['fused']['p95_m'] == pytest.approx(np.percentile(ramp, 95), abs=0.01)
    assert during['fused']['max_m'] == pytest.approx(ramp[-1], abs=0.01)
    assert during['autopilot']['median_m'] == pytest.approx(5.0, abs=0.02)
    assert result['phases']['before']['fused']['max_m'] == pytest.approx(0.1, abs=0.01)
    assert result['phases']['after']['autopilot']['median_m'] == pytest.approx(0.3, abs=0.01)
    denial = result['denial']
    assert denial['denied_sim_s'] == 150.0
    assert denial['denied_wall_s'] == pytest.approx(150.0 / RTF)
    assert denial['real_time_factor'] == RTF
    assert denial['start_after_cruise_sim_s'] == 60.0


def test_phases_are_cut_at_the_denier_events_on_the_simulation_clock():
    rows, events = make_run()
    # A huge error before cruise is in no phase.
    rows.append(fix('fused', CRUISE - 10, (0.0, 500.0)))
    result = score(rows, events)
    phases = result['phases']
    assert phases['before']['duration_sim_s'] == pytest.approx(OFF - CRUISE, abs=0.01)
    assert phases['during']['duration_sim_s'] == pytest.approx(ON - OFF, abs=0.01)
    assert phases['after']['duration_sim_s'] == pytest.approx(DONE - ON, abs=0.01)
    assert phases['before']['fused']['max_m'] < 1.0
    assert phases['before']['fused']['n'] == round((OFF - CRUISE) / 0.2)
    assert phases['after']['fused']['n'] == round((DONE - ON) / 0.2)


def test_camera_fixes_use_the_truth_at_their_stamp_corrections_at_arrival():
    result = score(*make_run())
    during = result['phases']['during']['fixes']
    # DemNav is stamped 0.5 s of wall time (0.1 s of simulation, 0.5 m flown)
    # before it arrives, on the truth's clock: scored against the truth at
    # the stamp, (3.5, 4) m, not at arrival, (3, 4) m.
    assert during['demnav']['median_m'] == pytest.approx(math.hypot(3.0 + SPEED * 0.5 * RTF, 4.0),
                                                         abs=0.01)
    assert during['correction']['median_m'] == pytest.approx(1.0, abs=0.01)
    assert during['wildnav']['n'] == 30


def test_fixes_stamped_on_another_clock_are_scored_at_arrival():
    rows, events = make_run()
    for row in rows:
        if row['k'] == 'demnav':
            row['t'] = 123.0   # simulation-time stamp against wall-clock truth
    during = score(rows, events)['phases']['during']['fixes']['demnav']
    assert during['median_m'] == pytest.approx(5.0, abs=0.01)


def test_invalid_camera_fixes_are_not_scored():
    rows, events = make_run()
    rows.append(fix('demnav', OFF + 10, (900.0, 0.0), status=-1))
    assert score(rows, events)['phases']['during']['fixes']['demnav']['max_m'] < 10.0


def test_covariance_consistency_shares():
    # DemNav's (3.5, 4) m error at its stamp against a 2.5 m sigma: 2.1 sigma.
    result = score(*make_run(demnav_error=(3.0, 4.0)))
    consistency = result['phases']['during']['fixes']['demnav']['consistency']
    assert consistency['sigma_median_m'] == 2.5
    assert consistency['beyond_2sigma'] == 1.0
    assert consistency['beyond_3sigma'] == 0.0
    fused = result['phases']['during']['fused']['consistency']
    # Ramp 0-10 m against 6 m sigma: never beyond 2 sigma.
    assert fused['beyond_2sigma'] == 0.0
    tight = score(*make_run(fused_sigma=2.0))['phases']['during']['fused']['consistency']
    # Beyond 4 m (2 sigma) for 60 % of the ramp, beyond 6 m for 40 %.
    assert tight['beyond_2sigma'] == pytest.approx(0.6, abs=0.01)
    assert tight['beyond_3sigma'] == pytest.approx(0.4, abs=0.01)


def test_mahalanobis_uses_the_full_horizontal_covariance():
    assert scorer.mahalanobis(3.0, 4.0, [1.0, 0.0, 1.0]) == pytest.approx(5.0)
    assert scorer.mahalanobis(2.0, 0.0, [4.0, 0.0, 1.0]) == pytest.approx(1.0)
    # Correlated: error along the long axis of an ellipse.
    cov = [5.0, 4.0, 5.0]          # sigma 3 along (1, 1), 1 along (1, -1)
    assert scorer.mahalanobis(3 / math.sqrt(2), 3 / math.sqrt(2), cov) == pytest.approx(1.0)
    assert scorer.mahalanobis(1 / math.sqrt(2), -1 / math.sqrt(2), cov) == pytest.approx(1.0)
    assert scorer.mahalanobis(1.0, 1.0, [0.0, 0.0, 0.0]) is None
    assert scorer.consistency([{'de': 1.0, 'dn': 0.0, 'cov': [0.0, 0.0, 0.0]}]) == {'n': 0}


def test_route_keeping_is_the_distance_from_the_commanded_leg():
    result = score(*make_run(route_offset=7.0))
    route = result['phases']['during']['route']
    # Each leg starts where the vehicle was and ends 7 m south of its line.
    assert 0.0 < route['median_m'] < 7.0
    assert route['waypoints_reached'] == 1
    assert route['flown_m'] == pytest.approx(SPEED * (ON - OFF), rel=0.01)
    exact = score(*make_run())['phases']['during']['route']
    assert exact['max_m'] == pytest.approx(0.0, abs=1e-6)
    assert not score(*make_run(route_offset=7.0), max_route_m=1.0)['checks']['route_keeping']


@pytest.mark.parametrize('change,check', [
    ({'during': lambda sim: 40.0}, 'fused_median'),
    ({'during': lambda sim: 40.0}, 'autopilot_p95'),
    ({'during': lambda sim: 30.0 if sim > ON - 2 else 1.0}, 'fused_max'),
    ({'before': 3.0}, 'before_on_gps'),
    ({'after': 5.0}, 'recovered'),
    ({'mode_during': 'LAND'}, 'stayed_guided'),
    ({'texts': [(OFF + 30, 'EKF Failsafe: changed to LAND Mode')]}, 'no_failsafe'),
    ({'gps1_lost': False}, 'gps1_lost'),
    ({'source': 'dead_reckoning'}, 'correction_coverage'),
    ({'on': OFF + 100}, 'denied_duration'),
    ({'fused_sigma': 1.0}, 'fused_consistency'),
])
def test_failures_are_detected(change, check):
    result = score(*make_run(**change))
    assert not result['checks'][check]
    assert not result['passed']


def test_failsafe_text_outside_the_denial_is_ignored():
    result = score(*make_run(texts=[(CRUISE, 'EKF variance'), (DONE - 5, 'GPS Glitch')]))
    assert result['checks']['no_failsafe']
    assert scorer.FAILSAFE_PATTERN.pattern == lite.FAILSAFE_PATTERN.pattern


def test_unusable_runs(tmp_path):
    rows, events = make_run()
    with pytest.raises(scorer.Unusable):
        scorer.score(rows, [e for e in events if e['event'] != 'gps_disabled'],
                     scorer.thresholds_for('isaac'))
    with pytest.raises(scorer.Unusable):
        scorer.score([r for r in rows if r['k'] != 'truth'], events,
                     scorer.thresholds_for('isaac'))
    with pytest.raises(scorer.Unusable):
        scorer.score([r for r in rows if r['k'] != 'meta'], events,
                     scorer.thresholds_for('isaac'))
    assert scorer.main([str(tmp_path)]) == 2


def test_cli_scores_a_compressed_run_and_takes_threshold_options(tmp_path, capsys):
    rows, events = make_run(during=lambda sim: 8.0)
    run_dir = write_run(tmp_path, rows, events, compress=True)
    with gzip.open(run_dir / 'recording.jsonl.gz', 'at') as stream:
        stream.write('{"k": "truth", "rx": 12')   # cut short when the recorder stopped
    assert scorer.main([str(run_dir)]) == 1      # the stack (isaac) comes from the recording
    summary = json.loads((run_dir / 'score.json').read_text())
    assert summary['stack'] == 'isaac'
    assert not summary['checks']['fused_median']
    assert 'GPS-DENIED SIM ISAAC FAIL' in capsys.readouterr().out
    assert scorer.main([str(run_dir), '--max-fused-median-m', '9',
                        '--max-autopilot-median-m', '9']) == 0
    assert scorer.main([str(run_dir), '--stack', 'gazebo']) == 0


def test_percentile_matches_numpy():
    values = sorted(np.random.default_rng(1).exponential(5.0, 333))
    for q in (0, 50, 95, 100):
        assert scorer.percentile(values, q) == pytest.approx(np.percentile(values, q))
    assert scorer.error_stats([]) == {'n': 0}


def test_projection_matches_the_synthetic_world():
    import synthetic_world
    assert scorer.METERS_PER_DEG_LAT == synthetic_world.METERS_PER_DEG_LAT
    assert scorer.latlon_to_local(37.93, -122.59, LAT0, LON0) == pytest.approx(
        synthetic_world.latlon_to_local(37.93, -122.59, LAT0, LON0))


# Fused error while denied (median, p95, max) of the committed navigation
# code: tools/gps_denied_sim_test.md, "Measured".
MEASURED = {'isaac': [(3.2, 5.5, 6.0), (2.6, 5.7, 6.0)],
            'gazebo': [(7.5, 11.2, 12.3), (5.7, 12.6, 14.5), (7.3, 19.8, 22.4),
                       (10.6, 13.7, 15.2), (7.8, 13.7, 15.8)]}


@pytest.mark.parametrize('stack', sorted(MEASURED))
def test_thresholds_pass_the_measured_runs_with_a_margin_that_can_still_fail(stack):
    thresholds = scorer.thresholds_for(stack)
    worst = [max(run[i] for run in MEASURED[stack]) for i in range(3)]
    limits = [thresholds[f'max_fused_{name}_m'] for name in ('median', 'p95', 'max')]
    for value, limit in zip(worst, limits):
        assert 1.5 * value <= limit <= 4.0 * value, (stack, value, limit)
    for name in ('median', 'p95', 'max'):
        assert thresholds[f'max_autopilot_{name}_m'] == thresholds[f'max_fused_{name}_m']


def test_every_stack_has_every_threshold():
    assert set(scorer.STACKS) == set(MEASURED)   # every stack's limits rest on measured runs
    keys = set(scorer.STACK_THRESHOLDS['gazebo'])
    for stack in scorer.STACKS:
        assert set(scorer.STACK_THRESHOLDS[stack]) == keys
        assert set(scorer.thresholds_for(stack)) == keys | set(scorer.COMMON_THRESHOLDS)
    with pytest.raises(ValueError):
        scorer.thresholds_for('lite')

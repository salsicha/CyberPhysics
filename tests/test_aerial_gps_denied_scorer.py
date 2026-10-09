"""Scorer of the Aerodrone GPS-denied SITL test (tools/gps_denied_sitl_scorer.py)."""
import json

import pytest

from conftest import REPO, load_script

scorer = load_script(REPO / 'tools/gps_denied_sitl_scorer.py', 'gps_denied_sitl_scorer_review')

LAT, LON = 37.9234, -122.5967
M_PER_DEG = 111319.49


def north(metres):
    return LAT + metres / M_PER_DEG


def telemetry(denied_s=300, fused_error_m=5.0, ekf_error_m=6.0, mode='GUIDED',
              source='demnav', texts=()):
    samples = []
    for t in range(denied_s + 1):
        samples.append({
            't': 100.0 + t, 'phase': 'denied', 'mode': mode, 'armed': True,
            'truth': [LAT, LON, 250.0], 'fused': [north(fused_error_m), LON, 250.0],
            'ekf': [north(ekf_error_m), LON, 250.0], 'correction_source': source})
    return {
        'samples': [{'t': 90.0, 'phase': 'gps', 'mode': 'GUIDED', 'armed': True,
                     'truth': [LAT, LON, 0], 'fused': [north(500.0), LON, 0],
                     'ekf': [LAT, LON, 0], 'correction_source': ''}] + samples,
        'events': [{'t': 99.5, 'event': 'gps_disabled'},
                   {'t': 101.0 + denied_s, 'event': 'gps_restored'}],
        'status_texts': [{'t': t, 'text': text} for t, text in texts],
        'completed': True,
    }


def run(data):
    return scorer.score(data, dict(scorer.DEFAULTS))


def test_good_run_passes_and_only_the_denied_phase_is_scored():
    result = run(telemetry())
    assert result['passed'], result['checks']
    fused = result['metrics']['fused']
    assert fused['rmse_m'] == pytest.approx(5.0, abs=0.05)
    assert fused['max_m'] == pytest.approx(5.0, abs=0.05)  # GPS-phase 500 m ignored
    assert result['metrics']['ekf']['rmse_m'] == pytest.approx(6.0, abs=0.05)
    assert result['metrics']['denied_duration_s'] == 300.0


@pytest.mark.parametrize('change,check', [
    (dict(fused_error_m=80.0), 'fused_rmse'),
    (dict(ekf_error_m=80.0), 'ekf_rmse'),
    (dict(mode='LAND'), 'stayed_guided'),
    (dict(source='dead_reckoning'), 'correction_coverage'),
    (dict(denied_s=60), 'denied_duration'),
    (dict(texts=[(150.0, 'EKF Failsafe: changed to LAND Mode')]), 'no_failsafe'),
])
def test_failures_are_detected(change, check):
    result = run(telemetry(**change))
    assert not result['checks'][check]
    assert not result['passed']


def test_failsafe_text_outside_the_denied_phase_is_ignored():
    result = run(telemetry(texts=[(50.0, 'EKF variance')]))
    assert result['checks']['no_failsafe']


def test_missing_estimates_are_unscored_not_zero_error():
    data = telemetry()
    for sample in data['samples'][1:20]:
        sample['fused'] = [float('nan'), LON, 0.0]
    result = run(data)
    assert result['metrics']['fused']['unscored'] == 19
    assert not result['checks']['fused_scored']


def test_aborted_run_fails(tmp_path):
    data = telemetry()
    data['completed'] = False
    data['error'] = 'takeoff rejected'
    path = tmp_path / 'telemetry.json'
    path.write_text(json.dumps(data))
    assert scorer.main([str(path)]) == 1
    assert scorer.main([str(tmp_path / 'missing.json')]) == 2


def test_thresholds_are_parameters(tmp_path):
    path = tmp_path / 'telemetry.json'
    path.write_text(json.dumps(telemetry(fused_error_m=30.0, ekf_error_m=3.0)))
    assert scorer.main([str(path)]) == 1
    assert scorer.main([str(path), '--max-fused-rmse-m', '35']) == 0


def test_default_thresholds_are_the_agreed_pass_criteria():
    defaults = scorer.DEFAULTS
    assert (defaults['max_fused_rmse_m'], defaults['max_fused_p95_m'],
            defaults['max_fused_max_m']) == (25.0, 45.0, 60.0)
    assert defaults['min_denied_s'] == 240.0
    assert defaults['min_guided_fraction'] == 1.0


def normal_flight(mode='GUIDED', texts=(), completed=True, fused_error_m=1.0):
    """GPS_DENIED_DENIED_S=0: the mission flies on GPS throughout."""
    samples = [{'t': 100.0 + t, 'phase': 'gps' if t < 300 else 'restored',
                'mode': mode, 'armed': True, 'truth': [LAT, LON, 250.0],
                'fused': [north(fused_error_m), LON, 250.0],
                'ekf': [north(0.5), LON, 250.0], 'correction_source': '',
                'waypoint': t // 60} for t in range(330)]
    return {'config': {'denied_s': 0.0}, 'samples': samples,
            'events': [{'t': 40.0, 'event': 'ready'}, {'t': 45.0, 'event': 'takeoff'},
                       {'t': 99.0, 'event': 'at_altitude'}],
            'status_texts': [{'t': t, 'text': text} for t, text in texts],
            'completed': completed}


def test_normal_flight_passes_without_a_denied_phase():
    # Pre-arm texts while the EKF initialises (before "ready") are normal.
    result = run(normal_flight(texts=[(10.0, 'PreArm: Need Position Estimate')]))
    assert result['mode'] == 'normal_flight'
    assert result['passed'], result['checks']
    assert result['metrics']['waypoints_reached'] == 5


@pytest.mark.parametrize('change,check', [
    (dict(texts=[(41.0, 'PreArm: GPS 2: Bad fix')]), 'no_prearm_gps2_or_failsafe_text'),
    (dict(texts=[(41.0, 'Arm: GPS 2: Bad fix')]), 'no_prearm_gps2_or_failsafe_text'),
    (dict(texts=[(200.0, 'EKF Failsafe: changed to LAND Mode')]),
     'no_prearm_gps2_or_failsafe_text'),
    (dict(mode='LAND'), 'stayed_guided'),
    (dict(completed=False), 'completed'),
    (dict(fused_error_m=40.0), 'fused_rmse'),
])
def test_normal_flight_failures_are_detected(change, check):
    result = run(normal_flight(**change))
    assert not result['checks'][check]
    assert not result['passed']

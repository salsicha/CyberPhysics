"""Aerodrone telemetry scorers: fused source labels, NaN metrics, test tagging
and correction-coverage gating."""
import json

import pytest

from conftest import REPO, load_script

robustness = load_script(
    REPO / 'applications/aerostack2/scripts/score_navigation_robustness.py',
    'score_navigation_robustness_review')
wilderness = load_script(
    REPO / 'applications/aerostack2/scripts/score_wilderness_mission.py',
    'score_wilderness_mission_review')

SUITE = json.loads(
    (REPO / 'applications/aerostack2/validation/navigation_robustness_tests.json')
    .read_text())
MISSIONS = json.loads(
    (REPO / 'applications/aerostack2/missions/wilderness_missions.json').read_text())


def _test(test_id):
    return next(test for test in SUITE['tests'] if test['id'] == test_id)


def _sample(test_id='gps_denied_valley_segment', **fields):
    sample = dict(
        test_id=test_id, horizontal_error_m=3.0, vertical_error_m=1.0,
        correction_source='demnav+wildnav', correction_confidence=0.6,
        demnav_valid=True, wildnav_valid=True, command_safe=True)
    sample.update(fields)
    return sample


def test_combined_fusion_source_is_the_contract_fused_source():
    result = robustness._score_test(
        SUITE, _test('gps_denied_valley_segment'), [_sample() for _ in range(5)])
    assert result['fallback_violations'] == 0
    assert result['pass_fail']['fallback_sources']
    assert result['passed']


def test_false_positives_are_counted_per_component_of_a_combined_source():
    samples = [_sample('low_texture_wildnav_rejection', wildnav_valid=False)]
    result = robustness._score_test(
        SUITE, _test('low_texture_wildnav_rejection'), samples)
    assert result['wildnav_false_positive_corrections'] == 1
    assert not result['pass_fail']['wildnav_false_positive_corrections']
    assert not result['passed']


def test_stale_cache_corrections_include_combined_sources():
    samples = [_sample('stale_dem_cache_rejected', stale_cache_used=True,
                       demnav_valid=False, wildnav_valid=True)]
    result = robustness._score_test(SUITE, _test('stale_dem_cache_rejected'), samples)
    assert result['stale_cache_corrections'] == 1
    assert not result['passed']


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), None, 'bogus'])
def test_non_finite_or_missing_errors_fail_instead_of_scoring_zero(bad):
    samples = [_sample(), _sample(horizontal_error_m=bad)]
    result = robustness._score_test(SUITE, _test('gps_denied_valley_segment'), samples)
    assert not result['pass_fail']['horizontal_error']
    assert result['invalid_metric_samples'] == 1
    assert not result['passed']


def test_untagged_test_does_not_score_other_tests_samples():
    samples = [_sample('gps_degraded_ridge_crossing') for _ in range(5)]
    result = robustness._score_test(SUITE, _test('gps_denied_valley_segment'), samples)
    assert result['samples'] == 0
    assert not result['passed']


def test_entirely_untagged_telemetry_scores_only_an_explicit_single_test():
    samples = [_sample(test_id=None) for _ in range(3)]
    for sample in samples:
        del sample['test_id']
    test = _test('gps_denied_valley_segment')
    assert robustness._score_test(SUITE, test, samples, allow_untagged=True)['samples'] == 3
    assert robustness._score_test(SUITE, test, samples)['samples'] == 0


def _mission(mission_id):
    return next(m for m in MISSIONS['missions'] if m['id'] == mission_id)


def _flight_samples(mission, source, confidence):
    samples = []
    for phase in mission['phases']:
        for waypoint in phase['waypoints']:
            x, y, z = waypoint['position_enu_m']
            samples.append(dict(
                position_enu_m=[x, y, z], agl_m=max(
                    40.0, float(waypoint['target_agl_m'])),
                correction_source=source, correction_confidence=confidence))
    return samples


def test_wilderness_mission_gates_correction_coverage():
    mission = _mission('ridge_crossing_gps_degraded')
    good = wilderness._score_mission(
        MISSIONS, mission, _flight_samples(mission, 'demnav+wildnav', 0.6))
    assert good['correction_coverage'] == 1.0
    assert good['pass_fail']['correction_coverage']
    assert good['passed']
    dead = wilderness._score_mission(
        MISSIONS, mission, _flight_samples(mission, 'dead_reckoning', 0.0))
    assert dead['correction_coverage'] == 0.0
    assert not dead['pass_fail']['correction_coverage']
    assert not dead['passed']


def test_wilderness_mission_without_visual_sources_does_not_require_corrections():
    mission = _mission('return_to_home')
    result = wilderness._score_mission(
        MISSIONS, mission, _flight_samples(mission, 'dead_reckoning', 0.0))
    assert 'correction_coverage' not in result['pass_fail']

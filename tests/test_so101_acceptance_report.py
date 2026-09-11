import json
import sys

import pytest
from conftest import REPO

sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
from so101_acceptance_report import DEFAULT_THRESHOLDS, acceptance_report


def complete_run():
    return dict(success=True, target_object='red_block', final_place_error_m=.01,
                collision_count=0, joint_limit_violations=0,
                command_saturation_fraction=.01, policy_latency_ms=50.,
                observation_age_ms=30., checks={'failed_grasp_recovery': True})


def test_complete_measurements_pass():
    assert acceptance_report([complete_run()], DEFAULT_THRESHOLDS)['success']


@pytest.mark.parametrize('field', ['final_place_error_m', 'collision_count',
    'joint_limit_violations', 'command_saturation_fraction', 'policy_latency_ms',
    'observation_age_ms', 'checks'])
def test_one_incomplete_run_cannot_be_hidden_by_complete_runs(field):
    missing = complete_run()
    del missing[field]
    report = acceptance_report([complete_run(), missing], DEFAULT_THRESHOLDS)
    assert not report['success']
    assert report['measurement_errors']
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('value', [None, float('nan'), float('inf'), -1, True, '0'])
def test_invalid_measurement_is_unverified(value):
    run = complete_run()
    run['policy_latency_ms'] = value
    report = acceptance_report([run], DEFAULT_THRESHOLDS)
    assert not report['checks']['policy_latency']
    assert report['metrics']['max_policy_latency_ms'] is None
    json.dumps(report, allow_nan=False)


def test_sparse_success_does_not_fabricate_safety_measurements():
    report = acceptance_report([{'success': True, 'final_place_error_m': 0}], DEFAULT_THRESHOLDS)
    assert not report['success']
    assert report['metrics']['collision_count_total'] is None
    assert report['metrics']['failed_grasp_recovery_rate'] is None


def test_null_checks_are_unverified():
    run = complete_run()
    run['checks'] = None
    report = acceptance_report([run], DEFAULT_THRESHOLDS)
    assert not report['success']
    assert report['metrics']['failed_grasp_recovery_rate'] is None


def test_failures_and_bad_counts_remain_failures():
    for field, value in [('collision_count', 1), ('joint_limit_violations', .5),
                         ('command_saturation_fraction', 2), ('success', 'false')]:
        run = complete_run()
        run[field] = value
        assert not acceptance_report([run], DEFAULT_THRESHOLDS)['success']
    with pytest.raises(ValueError, match='At least one run'):
        acceptance_report([], DEFAULT_THRESHOLDS)

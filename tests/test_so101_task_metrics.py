"""SO-101 task metrics must be measured, not assumed.

The Isaac scene cannot detect collisions, and the scripted demo plans from
simulator object poses, so metrics must say "unmeasured" / "used ground truth"
instead of reporting a clean run. Bridge command reports and simulated joint
states provide the latency, observation-age, saturation and joint-limit
measurements the acceptance report requires.
"""
import copy
import json
import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

from conftest import REPO

sys.modules.setdefault('pxr', MagicMock())
sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
sys.path.insert(0, str(REPO / 'applications/groot/scripts'))
from score_picking_task import score
from so101_acceptance_report import DEFAULT_THRESHOLDS, acceptance_report
from so101_common import UPPER_LIMITS
from so101_task_policy import build_pick_place_plan
from so101_task_scene import SO101TaskScene
from test_so101_task_demo import completed_telemetry, load_scenario

COMMAND_REPORT = {'policy_latency_ms': 35.0, 'observation_age_ms': 60.0,
                  'command_saturated': False, 'policy_used_ground_truth': False}


def measured_telemetry():
    scenario, telemetry = completed_telemetry()
    telemetry.update(joint_limit_violations=0, policy_commands={
        'count': 40, 'saturated': 1, 'max_policy_latency_ms': 35.0,
        'max_observation_age_ms': 60.0})
    return scenario, telemetry


def test_fully_measured_run_feeds_the_acceptance_report():
    scenario, telemetry = measured_telemetry()
    metrics = score(scenario, telemetry)
    assert metrics['success'] and metrics['unmeasured_checks'] == []
    assert metrics['command_saturation_fraction'] == 1 / 40
    assert metrics['policy_latency_ms'] == 35.0 and metrics['observation_age_ms'] == 60.0
    report = acceptance_report([metrics], DEFAULT_THRESHOLDS)
    assert report['success'], report['measurement_errors']


def test_unmeasured_collisions_and_ground_truth_are_not_reported_as_passing():
    scenario, telemetry = measured_telemetry()
    telemetry['collisions'] = None
    telemetry['policy_used_ground_truth'] = None
    for sample in telemetry['samples']:
        del sample['policy_used_ground_truth']
    metrics = score(scenario, telemetry)
    assert metrics['task_completed'] is True
    assert metrics['success'] is False
    assert metrics['collision_count'] is None
    assert set(metrics['unmeasured_checks']) == {'collision_free', 'no_policy_ground_truth'}
    assert 'collision_free' not in metrics['checks']
    report = acceptance_report([metrics], DEFAULT_THRESHOLDS)
    assert not report['success']
    assert any('collision_count' in error for error in report['measurement_errors'])


def test_ground_truth_use_fails_the_run_but_not_the_physical_task():
    scenario, telemetry = measured_telemetry()
    telemetry['policy_used_ground_truth'] = True
    metrics = score(scenario, telemetry)
    assert metrics['task_completed'] is True
    assert metrics['success'] is False
    assert metrics['checks']['no_policy_ground_truth'] is False
    assert metrics['policy_used_ground_truth'] is True


def drive_demo_episode(scene, scenario):
    plan = build_pick_place_plan(scenario)
    now = 0.0
    for waypoint in plan:
        for _ in range(30):
            scene.update(waypoint.joints, True, now=now)
            now += 0.05
    return now


def test_demo_episode_is_solved_but_flagged_as_ground_truth(tmp_path):
    scenario = load_scenario()
    task_id = scenario['tasks'][0]['id']
    scenario_with_task = copy.deepcopy(scenario)
    scenario_with_task['selected_task'] = scenario['tasks'][0]
    telemetry_path = tmp_path / 'so101_task_telemetry.json'
    scene = SO101TaskScene(MagicMock(), scenario, task_id, telemetry_path=str(telemetry_path))
    for _ in range(5):
        scene.record_policy_command({**COMMAND_REPORT, 'policy_used_ground_truth': True})
    scene.record_policy_command({**COMMAND_REPORT, 'command_saturated': True,
                                 'policy_latency_ms': 80.0, 'observation_age_ms': None})

    drive_demo_episode(scene, scenario_with_task)

    assert scene.status == 'solved'
    # Released standing on the bin, not hanging at the jaw centre.
    destination = scene.destination
    resting_z = destination['xyz'][2] + 0.5 * (destination['size'][2] + scene.target['size'][2])
    assert scene.target_position[2] == pytest.approx(resting_z, abs=1e-3)
    metrics = json.loads((tmp_path / 'so101_task_telemetry_metrics.json').read_text())
    assert metrics['task_completed'] is True
    assert metrics['success'] is False
    assert metrics['policy_used_ground_truth'] is True
    assert metrics['checks']['no_policy_ground_truth'] is False
    assert metrics['unmeasured_checks'] == ['collision_free']
    assert metrics['collision_count'] is None
    assert metrics['joint_limit_violations'] == 0
    assert metrics['command_saturation_fraction'] == 1 / 6
    assert metrics['policy_latency_ms'] == 80.0 and metrics['observation_age_ms'] == 60.0


def test_joint_limit_violations_are_counted(tmp_path):
    scenario = load_scenario()
    scene = SO101TaskScene(MagicMock(), scenario, scenario['tasks'][0]['id'])
    scene.update(np.zeros(6), True, now=0.0)
    beyond = UPPER_LIMITS.copy()
    beyond[1] += 0.05
    scene.update(beyond, True, now=0.1)
    scene.update(UPPER_LIMITS, True, now=0.2)  # at the limit is not a violation
    assert scene.telemetry(0.3)['joint_limit_violations'] == 1


def test_unsolved_run_still_writes_metrics_on_close(tmp_path):
    scenario = load_scenario()
    telemetry_path = tmp_path / 'run.json'
    scene = SO101TaskScene(MagicMock(), scenario, scenario['tasks'][0]['id'],
                           telemetry_path=str(telemetry_path))
    scene.update(np.zeros(6), True, now=0.0)
    scene.close()
    metrics = json.loads((tmp_path / 'run_metrics.json').read_text())
    assert metrics['task_completed'] is False and metrics['success'] is False

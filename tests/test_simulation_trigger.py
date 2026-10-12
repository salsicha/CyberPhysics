import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'applications/aerostack2/scripts'))
from simulation_trigger import SimulationTrigger


def ready(**changes):
    return dict(stamp=10.0, target_id='car1', launcher='ready', fire_block='', locked=True, **changes)


@pytest.mark.parametrize('action', ['fire', 'tag'])
def test_one_request_per_ready_identity_and_next_target(action):
    operator = SimulationTrigger(action)
    assert operator.command(ready(), 10.1) == {'action': action, 'target_id': 'car1'}
    assert operator.command(ready(), 10.2) is None
    status = ready()
    status['target_id'] = 'car2'
    assert operator.command(status, 10.2)['target_id'] == 'car2'


@pytest.mark.parametrize('changes', [
    {'stamp': 9.0}, {'stamp': 11.0}, {'stamp': float('nan')},
    {'stamp': None}, {'launcher': 'disarmed'}, {'launcher': 'fault'},
    {'fire_block': 'not centered'}, {'target_id': ''}, {'tagged_ids': ['car1']},
])
def test_gates_never_issue_fire(changes):
    status = ready()
    status.update(changes)
    assert SimulationTrigger().command(status, 10.1) is None


def test_turret_also_requires_lock():
    status = ready()
    status['locked'] = False
    assert SimulationTrigger('tag').command(status, 10.1) is None


def test_physical_tracker_never_imports_the_simulated_operator():
    root = Path(__file__).parents[1]
    for source in ['applications/aerostack2/scripts/aerodrone_tracker.py',
                   'applications/pantiltros/scripts/pantilt_yolo_tracker.py']:
        assert 'simulation_trigger' not in (root / source).read_text()

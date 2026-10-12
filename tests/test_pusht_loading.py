"""Legacy policy evaluation must retain its trained normalization statistics."""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    'pusht_example', Path(__file__).resolve().parents[1] /
    'applications/so101/scripts/evaluate_pretrained_policy.py')
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


@pytest.fixture
def checkpoint(monkeypatch):
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(isfinite=np.isfinite, equal=np.array_equal))
    config = SimpleNamespace(
        input_features={'observation.state': SimpleNamespace(type='STATE')},
        output_features={'action': SimpleNamespace(type='ACTION')},
        normalization_mapping={key: SimpleNamespace(value='MIN_MAX') for key in ('STATE', 'ACTION')})
    weights = {
        f'{prefix}.buffer_{key}.{name}': np.array([value, value + 1.])
        for prefix, key in [('normalize_inputs', 'observation_state'),
                            ('normalize_targets', 'action'), ('unnormalize_outputs', 'action')]
        for name, value in [('min', 10.), ('max', 400.)]
    }
    return config, weights


def test_recovers_training_scale_without_identity_defaults(checkpoint):
    config, weights = checkpoint
    stats = example.legacy_statistics(weights, config)
    np.testing.assert_array_equal(stats['action']['max'], [400., 401.])
    np.testing.assert_array_equal(stats['observation.state']['min'], [10., 11.])


def test_rejects_missing_or_nonfinite_statistics(checkpoint):
    config, weights = checkpoint
    weights.pop('normalize_inputs.buffer_observation_state.min')
    with pytest.raises(KeyError):
        example.legacy_statistics(weights, config)
    weights['normalize_inputs.buffer_observation_state.min'] = np.array([np.nan, 1.])
    with pytest.raises(ValueError, match='Invalid checkpoint statistic'):
        example.legacy_statistics(weights, config)


def test_action_input_and_output_scales_must_agree(checkpoint):
    config, weights = checkpoint
    weights['unnormalize_outputs.buffer_action.max'] = np.array([100., 101.])
    with pytest.raises(ValueError, match='statistics disagree'):
        example.legacy_statistics(weights, config)

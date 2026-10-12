"""The inference example must never treat missing weights as pretrained success."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'pi0_example', Path(__file__).resolve().parents[1] / 'applications/so101/scripts/pi0_example.py')
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


def test_pi0_load_is_strict_and_propagates_failure():
    def reject(weights, *, strict):
        assert strict is True
        assert weights == {'model.weight': 123, 'model.bias': 456}
        raise RuntimeError('missing pretrained tensor')

    policy = SimpleNamespace(config=None, _fix_pytorch_state_dict_keys=lambda state, config: state,
                             load_state_dict=reject)
    with pytest.raises(RuntimeError, match='missing pretrained tensor'):
        example.load_weights_strict(policy, {'weight': 123, 'model.bias': 456})

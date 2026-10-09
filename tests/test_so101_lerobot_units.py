"""LeRobot SO-101 units must be explicit when preparing GR00T datasets.

so101_follower defaults to use_degrees=False (-100..100 of each joint's
calibrated travel), which looks like plausible degrees after deg2rad; and
LeRobot's zero pose is the calibration mid-range, not this URDF's zero.
"""
from unittest.mock import MagicMock
import json
import sys

import numpy as np
import pytest

from conftest import REPO, load_script

# The converter imports pyarrow for parquet I/O; the logic under test does not.
for name in ('pyarrow', 'pyarrow.parquet'):
    sys.modules.setdefault(name, MagicMock())
normalize = load_script(REPO / 'applications/groot/scripts/normalize_so101_dataset.py',
                        'groot_normalize_so101_dataset')
MAPPING_FILE = REPO / 'systems/so101/config/lerobot_joint_mapping.json'
DEGREE_STEP = 180.0 / 4095.0


def lerobot_degrees(rng, count=200):
    """use_degrees=True values: (raw - mid) * 360 / 4095 for integer raw ticks."""
    ticks = rng.integers(-1100, 1100, size=(count, 5)) + rng.integers(0, 2, size=(1, 5)) * 0.5
    return ticks * 2 * DEGREE_STEP


def lerobot_range(rng, count=200):
    """use_degrees=False values for a 2600-tick calibrated range, clipped at +/-100."""
    raw = rng.integers(0, 2601, size=(count, 5))
    raw[0] = 0
    raw[1] = 2600
    return raw / 2600 * 200 - 100


def test_normalised_range_data_is_detected_and_degrees_are_confirmed():
    rng = np.random.default_rng(3)
    assert normalize.classify_lerobot_arm_units(lerobot_range(rng)) == 'range_m100_100'
    assert normalize.classify_lerobot_arm_units(lerobot_degrees(rng)) == 'degrees'
    assert normalize.classify_lerobot_arm_units(np.array([[150.0, 0, 0, 0, 0]])) == 'degrees'
    assert normalize.classify_lerobot_arm_units(np.array([[12.345, 0, 0, 0, 0]])) == 'unknown'


def gripper_column(rows):
    return np.linspace(0.0, 100.0, rows)[:, None]


def test_joint_mapping_offsets_and_signs_are_applied():
    mapping = {name: {'sign': 1.0, 'offset_deg': 0.0} for name in normalize.JOINT_NAMES[:5]}
    mapping['elbow_flex'] = {'sign': -1.0, 'offset_deg': 45.0}
    degrees = np.array([[10.0, -20.0, 30.0, 0.0, 5.0], [0.0, 0.0, 0.0, 0.0, 0.0]])
    matrix = np.hstack([degrees, gripper_column(2)])
    output = normalize.transform_matrix(
        matrix, input_units='degrees', gripper_closed_value=0.0, gripper_open_value=100.0,
        joint_mapping=mapping)
    expected = np.deg2rad([[10.0, -20.0, 15.0, 0.0, 5.0], [0.0, 0.0, 45.0, 0.0, 0.0]])
    np.testing.assert_allclose(output[:, :5], expected, atol=1e-6)
    np.testing.assert_allclose(output[:, 5], [0.0, 0.04], atol=1e-7)


def test_mapped_values_outside_urdf_limits_are_rejected():
    mapping = {name: {'sign': 1.0, 'offset_deg': 0.0} for name in normalize.JOINT_NAMES[:5]}
    mapping['shoulder_lift'] = {'sign': 1.0, 'offset_deg': 90.0}  # wrong offset
    matrix = np.hstack([np.array([[0.0, 60.0, 0.0, 0.0, 0.0]]), gripper_column(1)])
    with pytest.raises(ValueError, match='shoulder_lift.*joint-mapping'):
        normalize.transform_matrix(matrix, input_units='degrees', gripper_closed_value=0.0,
                                   gripper_open_value=100.0, joint_mapping=mapping)


def test_repository_mapping_file_is_the_nominal_calibration():
    """Offsets from so101_new_calib.urdf (zero = LeRobot mid-range); see the file's comment."""
    mapping = normalize.load_joint_mapping(MAPPING_FILE)
    assert not normalize.is_identity_mapping(mapping)
    data = json.loads(MAPPING_FILE.read_text())
    assert set(data['joints']) == set(normalize.JOINT_NAMES[:5])
    expected = {'shoulder_pan': (-1.0, 0.0), 'shoulder_lift': (1.0, 14.0),
                'elbow_flex': (1.0, 73.9), 'wrist_flex': (1.0, 2.1), 'wrist_roll': (-1.0, 0.0)}
    assert {name: (entry['sign'], entry['offset_deg']) for name, entry in mapping.items()} == expected
    # LeRobot's calibration pose (all zeros, gripper closed) maps inside the URDF limits.
    zero = np.hstack([np.zeros((1, 5)), gripper_column(1)])
    output = normalize.transform_matrix(zero, input_units='degrees', gripper_closed_value=0.0,
                                        gripper_open_value=100.0, joint_mapping=mapping)
    np.testing.assert_allclose(output[0, :5], np.deg2rad([0.0, 14.0, 73.9, 2.1, 0.0]), atol=1e-6)
    bad = MAPPING_FILE.parent / 'does_not_exist.json'
    with pytest.raises(FileNotFoundError):
        normalize.load_joint_mapping(bad)

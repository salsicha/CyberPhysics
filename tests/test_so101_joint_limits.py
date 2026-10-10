"""SO-101 joint limits: one source of truth, consistent with the arm's geometry.

This URDF's zero is the arm pointing straight up, while TheRobotStudio's
so101_new_calib.urdf (whose limits describe the physical range) has its zero at
LeRobot's calibrated mid-range pose. systems/so101/config/joint_limits.json
holds those limits mapped into this URDF with the nominal LeRobot joint
mapping; the URDF files, simulators, safety gates, GR00T scripts and SmolVLA
server must all use it, and LeRobot data at the ends of its calibrated range
must map inside it.
"""
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock

import numpy as np
import pytest
from conftest import REPO, load_script

sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
sys.path.insert(0, str(REPO / 'applications/groot/scripts'))
import so101_common as common
import so101_joint_limits as groot_limits
import so101_task_policy as task_policy
from validate_calibration_and_dataset import validate_joint_limits

# The converter imports pyarrow for parquet I/O; the logic under test does not.
for _name in ('pyarrow', 'pyarrow.parquet'):
    sys.modules.setdefault(_name, MagicMock())
normalize = load_script(REPO / 'applications/groot/scripts/normalize_so101_dataset.py',
                        'groot_normalize_so101_dataset')
_spec = importlib.util.spec_from_file_location(
    'smolvla_so101_policy_server', REPO / 'applications/smolvla/scripts/so101_policy_server.py')
smolvla = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(smolvla)

LIMITS_FILE = REPO / 'systems/so101/config/joint_limits.json'
MAPPING_FILE = REPO / 'systems/so101/config/lerobot_joint_mapping.json'
URDF = REPO / 'systems/so101/urdf/so101.urdf'
XACRO = REPO / 'systems/so101/urdf/so101.urdf.xacro'
SCENARIO = REPO / 'systems/so101/scenarios/picking_table.json'
ARM = common.JOINT_NAMES[:5]
# TheRobotStudio Simulation/SO101/so101_new_calib.urdf <limit> values (rad).
SO101_NEW_CALIB = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
}
# The limits this repository used before, copied from so101_new_calib without
# moving them to the straight-up zero.
OLD_SYMMETRIC_LIMITS = np.array([1.9199, 1.7453, 1.7453, 1.6581, 2.7439])


def source_of_truth():
    joints = json.loads(LIMITS_FILE.read_text())['joints']
    return np.array([[joints[name]['lower'], joints[name]['upper']] for name in common.JOINT_NAMES])


def nominal_mapping():
    return json.loads(MAPPING_FILE.read_text())['joints']


def lerobot_extremes_deg():
    """LeRobot use_degrees=True readings at both ends of the calibrated range."""
    low = np.array([math.degrees(SO101_NEW_CALIB[name][0]) for name in ARM])
    high = np.array([math.degrees(SO101_NEW_CALIB[name][1]) for name in ARM])
    return low, high


def test_limits_are_so101_new_calib_mapped_into_this_urdf():
    joints = json.loads(LIMITS_FILE.read_text())['joints']
    mapping = nominal_mapping()
    for name in ARM:
        assert tuple(joints[name]['so101_new_calib_rad']) == SO101_NEW_CALIB[name]
        sign = mapping[name]['sign']
        offset = math.radians(mapping[name]['offset_deg'])
        lower, upper = sorted(sign * value + offset for value in SO101_NEW_CALIB[name])
        # Rounded outward to 1e-5 rad, so the physical range stays inside.
        assert lower - 1e-5 <= joints[name]['lower'] <= lower, name
        assert upper <= joints[name]['upper'] <= upper + 1e-5, name
    assert (joints['gripper']['lower'], joints['gripper']['upper']) == (0.0, 0.04)
    # Spot values in degrees (README): shoulder_lift -86..114, elbow -22.9..170.7.
    np.testing.assert_allclose(np.degrees([joints['shoulder_lift']['lower'], joints['shoulder_lift']['upper']]),
                               [-86.0, 114.0], atol=1e-3)
    np.testing.assert_allclose(np.degrees([joints['elbow_flex']['lower'], joints['elbow_flex']['upper']]),
                               [-22.93, 170.73], atol=1e-2)


def test_every_python_consumer_loads_the_source_of_truth():
    limits = source_of_truth()
    np.testing.assert_array_equal(common.JOINT_LIMITS, limits)
    np.testing.assert_array_equal(common.LOWER_LIMITS, limits[:, 0])
    np.testing.assert_array_equal(common.UPPER_LIMITS, limits[:, 1])
    assert groot_limits.JOINT_NAMES == common.JOINT_NAMES
    np.testing.assert_array_equal(groot_limits.JOINT_LIMITS, limits)
    np.testing.assert_array_equal(task_policy.JOINT_LIMITS, limits.astype(np.float32))
    np.testing.assert_array_equal(normalize.ARM_JOINT_LIMITS, limits[:5])
    np.testing.assert_array_equal(smolvla.load_urdf_arm_limits(), limits[:5])
    assert common.joint_limits_file() == LIMITS_FILE
    assert groot_limits.joint_limits_file() == LIMITS_FILE


def test_limits_file_can_be_overridden_and_must_be_ordered(tmp_path, monkeypatch):
    data = json.loads(LIMITS_FILE.read_text())
    data['joints']['elbow_flex']['lower'] = 3.0
    broken = tmp_path / 'joint_limits.json'
    broken.write_text(json.dumps(data))
    monkeypatch.setenv(common.JOINT_LIMITS_FILE_ENV, str(broken))
    assert common.joint_limits_file() == broken
    for load in (common.load_joint_limits, groot_limits.load_joint_limits, smolvla.load_urdf_arm_limits):
        with pytest.raises(ValueError, match='lower < upper'):
            load()


def test_no_python_consumer_hard_codes_limit_values():
    literals = {'1.9199', '1.7453', '1.6581', '2.7439', '1.50099', '1.98968', '0.40021',
                '2.9798', '1.62141', '1.69472', '2.84121', '2.74385'}
    sources = []
    for directory in ('applications/so101/scripts', 'applications/groot/scripts',
                      'applications/smolvla/scripts'):
        sources += sorted((REPO / directory).glob('*.py'))
    assert sources
    for path in sources:
        text = path.read_text()
        assert not [value for value in literals if value in text], path


def urdf_limits(path):
    joints = {joint.attrib['name']: joint for joint in ET.parse(path).getroot().findall('joint')}
    return np.array([[float(joints[name].find('limit').attrib[key]) for key in ('lower', 'upper')]
                     for name in common.JOINT_NAMES])


def test_both_urdf_files_use_the_source_of_truth():
    np.testing.assert_array_equal(urdf_limits(URDF), source_of_truth())
    # The Xacro has no limit numbers of its own: every movable joint takes its
    # <limit> from joint_limits.json through the joint_limit macro.
    xacro_ns = '{http://www.ros.org/wiki/xacro}'
    root = ET.parse(XACRO).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    for name in common.JOINT_NAMES:
        assert joints[name].find('limit') is None, name
        macro = joints[name].find(f'{xacro_ns}joint_limit')
        assert macro is not None and macro.attrib['name'] == name, name
    load = next(prop for prop in root.findall(f'{xacro_ns}property')
                if prop.attrib['name'] == 'joint_limits').attrib['value']
    relative = load.split("load_yaml('")[1].split("'")[0]
    assert (XACRO.parent / relative).resolve() == LIMITS_FILE


def test_validator_checks_urdf_limits_against_the_source_of_truth(tmp_path):
    assert validate_joint_limits(URDF)['max_error'] == 0.0
    tree = ET.parse(URDF)
    elbow = next(joint for joint in tree.getroot().findall('joint') if joint.attrib['name'] == 'elbow_flex')
    elbow.find('limit').attrib.update(lower='-1.7453', upper='1.7453')
    stale = tmp_path / 'so101.urdf'
    tree.write(stale)
    with pytest.raises(AssertionError, match='elbow_flex limits'):
        validate_joint_limits(stale)


def test_lerobot_calibration_extremes_map_inside_the_urdf_limits():
    """LeRobot readings at +/- its calibrated range land inside the URDF limits."""
    low, high = lerobot_extremes_deg()
    mapping = nominal_mapping()
    rows = np.array([low, high, [low[0], high[1], low[2], high[3], low[4]],
                     [high[0], low[1], high[2], low[3], high[4]]])
    matrix = np.hstack([rows, np.array([[0.0], [100.0], [50.0], [25.0]])])
    converted = normalize.transform_matrix(
        matrix, input_units='degrees', gripper_closed_value=0.0, gripper_open_value=100.0,
        joint_mapping=normalize.load_joint_mapping(MAPPING_FILE))
    limits = source_of_truth()
    # float32 datasets: allow its rounding (1e-7 rad) at the exact pan/roll limits.
    assert np.all(converted[:, :5] >= limits[:5, 0] - 1e-6)
    assert np.all(converted[:, :5] <= limits[:5, 1] + 1e-6)
    # Both ends of every joint's range are reached, i.e. the limits are tight.
    np.testing.assert_allclose(converted[:, :5].min(axis=0), limits[:5, 0], atol=2e-5)
    np.testing.assert_allclose(converted[:, :5].max(axis=0), limits[:5, 1], atol=2e-5)
    # The simulators and safety gates pass them through unclamped.
    for row in converted.astype(np.float64):
        np.testing.assert_allclose(common.validated_joint_command(row), row, atol=1e-6)
    # The SmolVLA server's LeRobot-to-URDF conversion agrees.
    smolvla_mapping = smolvla.load_joint_mapping(str(MAPPING_FILE))
    for row in matrix:
        isaac = smolvla.so101_to_isaac_units(row.astype(np.float32), smolvla_mapping)
        assert np.all(isaac[:5] >= limits[:5, 0] - 1e-6) and np.all(isaac[:5] <= limits[:5, 1] + 1e-6)
    assert smolvla.joints_outside_urdf_limits(low, high, smolvla_mapping, limits[:5]) == []
    # The old symmetric limits rejected the real arm's shoulder and elbow range.
    mapped = np.deg2rad(np.array([mapping[name]['sign'] for name in ARM]) * rows
                        + np.array([mapping[name]['offset_deg'] for name in ARM]))
    beyond_old = np.abs(mapped) > OLD_SYMMETRIC_LIMITS + normalize.ARM_LIMIT_TOLERANCE_RAD
    assert [ARM[index] for index in np.flatnonzero(beyond_old.any(axis=0))] == [
        'shoulder_lift', 'elbow_flex']


def test_simulators_cannot_reach_poses_the_arm_cannot():
    beyond = np.zeros(len(common.JOINT_NAMES))
    beyond[2] = math.radians(-100.0)  # elbow bent backwards: allowed by the old limits
    clamped = common.validated_joint_command(beyond)
    assert clamped[2] == pytest.approx(common.LOWER_LIMITS[2])
    assert math.degrees(clamped[2]) == pytest.approx(-22.93, abs=0.01)


def test_smolvla_flags_a_checkpoint_range_that_does_not_fit_the_urdf():
    low, high = lerobot_extremes_deg()
    identity = smolvla.load_joint_mapping(None)
    outside = smolvla.joints_outside_urdf_limits(low, high, identity, source_of_truth()[:5])
    assert outside == ['shoulder_lift', 'elbow_flex']


def test_scripted_demo_stays_inside_the_limits():
    scenario = json.loads(SCENARIO.read_text())
    limits = source_of_truth()
    for task in scenario['tasks']:
        scenario['selected_task'] = task
        plan = task_policy.build_pick_place_plan(scenario)
        for waypoint in plan:
            assert np.all(waypoint.joints >= limits[:, 0]) and np.all(waypoint.joints <= limits[:, 1]), \
                (task['id'], waypoint.name)
        controller = task_policy.ScriptedPickPlaceController(scenario=scenario)
        current = np.zeros(len(common.JOINT_NAMES), dtype=np.float32)
        for _ in range(150):
            current = controller.get_action({'state': {'joint_positions': current.reshape(1, 1, -1)}})
            assert np.all(current >= limits[:, 0].astype(np.float32))
            assert np.all(current <= limits[:, 1].astype(np.float32))
            if controller.status == 'solved':
                break
        assert controller.status == 'solved', task['id']

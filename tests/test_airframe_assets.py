"""Both flight backends use the same traceable CAD assembly and propulsion."""
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from xml.etree import ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
DIRECTORY = ROOT / 'applications/aerostack2/models/aerodrone'
DATA = json.loads((DIRECTORY / 'airframe.json').read_text())
sys.path.insert(0, str(ROOT / 'applications/ardupilot_sitl/scripts'))
from airframe_model import load_visual_triangles


def test_visual_loader_rejects_lfs_pointer_and_truncated_mesh(tmp_path):
    path = tmp_path / 'mesh.stl'
    path.write_text('version https://git-lfs.github.com/spec/v1\noid sha256:123\nsize 134\n')
    with pytest.raises(RuntimeError, match='git lfs pull'):
        load_visual_triangles(path)
    path.write_bytes(b'\0' * 80 + struct.pack('<I', 1))
    with pytest.raises(ValueError, match='Invalid binary'):
        load_visual_triangles(path)
    triangle = struct.pack('<12fH', 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
    path.write_bytes(path.read_bytes() + triangle)
    assert load_visual_triangles(path).tolist() == [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]]


def test_mass_budget_and_inertia_are_physical():
    assert DATA['mass_kg'] == pytest.approx(sum(item['mass_kg'] for item in DATA['items']))
    assert DATA['mass_kg'] == pytest.approx(3.17, abs=.01)
    center = sum(item['mass_kg'] * np.array(item['center_m']) for item in DATA['items']) / DATA['mass_kg']
    assert np.allclose(center, DATA['center_of_mass_m'])
    tensor = np.array(DATA['inertia_kg_m2'])
    assert np.allclose(tensor, tensor.T)
    values = np.linalg.eigvalsh(tensor)
    assert min(values) > 0 and max(values) < sum(values) - max(values)


def test_airframe_uses_current_printable_stls():
    assert len(DATA['stl_sha256']) > 20
    for name, expected in DATA['stl_sha256'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    for name, expected in DATA['source_sha256'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name


def test_gazebo_mass_rotor_order_and_propulsion_match_manifest():
    model = ET.parse(DIRECTORY / 'model.sdf')
    # Camera frames contribute another two 1 mg links in the template.
    total = sum(float(e.text) for e in model.findall('.//inertial/mass')) + 2e-6
    assert total == pytest.approx(DATA['mass_kg'], abs=1e-8)
    for i, position in enumerate(DATA['rotor_positions_flu_m']):
        pose = model.find(f'.//link[@name="rotor_{i}"]/pose')
        assert list(map(float, pose.text.split()[:3])) == pytest.approx(position)
        assert math.hypot(*position[:2]) == pytest.approx(.25)
    assert DATA['motor_constant'] * DATA['max_rotor_speed_rad_s'] ** 2 == pytest.approx(1.6 * 9.80665)
    template = (ROOT / 'applications/gazebo/models/quadrotor_ardupilot.sdf.jinja').read_text()
    import re
    assert float(re.search(r'<maxRotVelocity>(.*?)<', template)[1]) == pytest.approx(DATA['max_rotor_speed_rad_s'])
    assert float(re.search(r'<motorConstant>(.*?)<', template)[1]) == pytest.approx(DATA['motor_constant'])


def test_camera_and_spawn_follow_the_assembly():
    assert DATA['camera_position_flu_m'] == pytest.approx([.14215, 0, -.0798])
    assert DATA['ground_clearance_m'] == pytest.approx(.3565)
    assert max(-box['center_m'][2] + box['size_m'][2] / 2 for box in DATA['collision_boxes']) == pytest.approx(DATA['ground_clearance_m'])

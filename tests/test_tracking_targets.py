"""Tracking targets of the Aerodrone simulations (systems/aerodrone/tracking.md):
routes resolved on the terrain the simulators render, the poses both
give the targets, the /simulation/targets ground truth, the Fuel model
conversion and the tracking scenario's mission tree."""
import hashlib
import io
import json
import math
import sys
import zipfile
from types import SimpleNamespace as NS
from xml.etree import ElementTree

import numpy as np
import pytest

from conftest import REPO, load_script

SCRIPTS = REPO / 'applications/demnav/scripts'
sys.path.insert(0, str(SCRIPTS))  # the builder imports both modules by name

tt = load_script(SCRIPTS / 'tracking_targets.py', 'tracking_targets')
tm = load_script(SCRIPTS / 'target_models.py', 'target_models')
sys.modules.setdefault('tracking_targets', tt)
sys.modules.setdefault('target_models', tm)
if 'terrain_collisions' not in sys.modules:
    sys.modules['terrain_collisions'] = load_script(
        SCRIPTS / 'terrain_collisions.py', 'terrain_collisions')
builder = load_script(SCRIPTS / 'build_georeferenced_terrain.py', 'build_terrain_tracking_review')

CONFIG = json.loads((REPO / 'applications/aerostack2/missions/tracking_targets.json').read_text())
MISSION = json.loads(
    (REPO / 'applications/aerostack2/missions/wilderness_missions.json').read_text())
HOME = MISSION['missions'][0]['phases'][0]['waypoints'][0]['position_enu_m'][:2]
CC0 = 'Creative Commons Zero v1.0 Universal'
# The simulated downward camera: Gazebo's and Isaac's 60 deg across
# 640 x 480.
NARROW_FOV_DEG, ASPECT = 60.0, 480 / 640


def plane(east, north):
    return 3.0 + 0.05 * np.asarray(east) + 0.02 * np.asarray(north)


def grid_mesh(height, half=200.0, count=11):
    """Vertices of a make_mesh-like grid (rows north to south)."""
    east = np.linspace(-half, half, count)
    north = np.linspace(half, -half, count)
    return np.array([(e, n, float(height(e, n))) for n in north for e in east])


# --- terrain ------------------------------------------------------------------

def triangle_height(triangles, east, north):
    """Height of the triangle under (east, north) by barycentric weights."""
    for a, b, c in triangles:
        m = np.array([[b[0] - a[0], c[0] - a[0]], [b[1] - a[1], c[1] - a[1]]])
        u, v = np.linalg.solve(m, [east - a[0], north - a[1]])
        if u >= -1e-9 and v >= -1e-9 and u + v <= 1 + 1e-9:
            return a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2])
    raise AssertionError('point outside the mesh')


def test_targets_stand_on_the_triangles_the_simulators_render(tmp_path):
    """MeshHeight interpolates terrain_mesh.npz over the faces make_mesh
    writes to terrain.obj (Gazebo's and Isaac's visual terrain)."""
    rng = np.random.default_rng(1)
    grid = (800.0 + rng.uniform(-30, 30, size=(9, 9))).astype(np.float32)
    args = NS(max_grid_size=257, area_km=0.8, lat=37.9234, lon=-122.5967, origin_alt=781.0,
              zoom=17)
    builder.make_mesh(grid, args, (0, 1, 0, 1, 512, 512), tmp_path)
    text = (tmp_path / 'terrain.obj').read_text().splitlines()
    vertices = np.array([[float(v) for v in line.split()[1:4]] for line in text
                         if line.startswith('v ')])
    faces = [[int(corner.split('/')[0]) - 1 for corner in line.split()[1:]]
             for line in text if line.startswith('f ')]
    height = tt.MeshHeight.load(tmp_path / 'terrain_mesh.npz')
    for east, north in rng.uniform(-399, 399, size=(200, 2)):
        expected = triangle_height(vertices[faces], east, north)
        assert height(east, north) == pytest.approx(expected, abs=2e-3)
    assert height(*vertices[10, :2]) == pytest.approx(vertices[10, 2], abs=1e-3)


# --- routes -----------------------------------------------------------------

def test_route_is_a_closed_loop_with_rounded_corners():
    corners = [[-14.0, -9.0], [14.0, -9.0], [14.0, 9.0], [-14.0, 9.0]]
    rows, length = tt.resample(tt.route_polyline(corners, 7.0, arc_step_deg=0.5), 0.2)
    assert length == pytest.approx(2 * (28 - 14) + 2 * (18 - 14) + 2 * math.pi * 7, abs=0.02)
    s, east, north, heading = rows.T
    assert np.allclose(np.diff(s), length / len(s))
    # It leaves the first corner heading for the second (east) ...
    assert (east[0], north[0], heading[0]) == pytest.approx((-7.0, -9.0, 0.0))
    # ... stays inside the corners' rectangle and turns left all the way round.
    assert np.all(np.abs(east) <= 14 + 1e-9) and np.all(np.abs(north) <= 9 + 1e-9)
    turns = np.diff(np.unwrap(heading))
    assert np.all(turns >= -1e-9) and np.unwrap(heading)[-1] == pytest.approx(
        2 * math.pi, abs=0.05)
    # Rounded corners: no sample in a corner's square beyond its arc.
    corner = np.hypot(np.abs(east) - 7, np.abs(north) - 2)
    outside = (np.abs(east) > 7) & (np.abs(north) > 2)
    assert np.all(corner[outside] <= 7 + 0.01)


def resolve(targets, height=plane, models=None):
    config = {'sample_spacing_m': 0.2,
              'models': models or {'car': {'tilt_with_ground': True, 'wheelbase_m': 2.5,
                                           'track_m': 1.5},
                                   'walker': {'tilt_with_ground': False}},
              'targets': targets}
    return tt.Tracks(tt.resolve_tracks(config, height, (-20.0, -30.0, 2.0)))


CAR = {'name': 'car_1', 'class': 'car', 'model': 'car', 'speed_mps': 5.0,
       'corner_radius_m': 7.0, 'route_enu_m': [[0, 0], [28, 0], [28, 18], [0, 18]]}


def test_targets_move_at_their_speed_on_the_ground_and_loop():
    tracks = resolve([CAR, {'name': 'person_2', 'class': 'person', 'model': 'walker',
                            'speed_mps': 0.0, 'heading_deg': 90.0,
                            'route_enu_m': [[5.0, 6.0]]}])
    car, person = tracks.tracks
    period = car.length / car.speed
    times = np.arange(0.0, 2 * period, 0.05)
    positions = np.array([[p.east, p.north, p.up] for p in (car.pose(t) for t in times)])
    # On the ground under it ...
    assert positions[:, 2] == pytest.approx(plane(positions[:, 0], positions[:, 1]), abs=1e-3)
    # ... at its speed (chords of 0.25 m on 7 m arcs) ...
    speed = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1) / 0.05
    assert speed == pytest.approx(5.0, rel=2e-3)
    # ... round and round.
    for t in (0.0, 3.3, 11.1):
        a, b = car.pose(t), car.pose(t + period)
        assert (a.east, a.north, a.up, a.yaw) == pytest.approx((b.east, b.north, b.up, b.yaw))
    standing = [person.pose(t) for t in (0.0, 7.0, 100.0)]
    assert {(p.east, p.north) for p in standing} == {(5.0, 6.0)}
    # Track samples keep 0.1 mm and 0.1 mrad.
    assert standing[0].yaw == pytest.approx(math.pi / 2, abs=1e-4)
    assert standing[0].up == pytest.approx(plane(5.0, 6.0), abs=1e-4)


def test_a_car_pitches_and_rolls_with_the_ground_a_person_stays_upright():
    tracks = resolve([CAR, dict(CAR, name='person_1', model='walker', speed_mps=1.5)])
    car, person = tracks.tracks
    east = car.pose(1.0)  # on the first leg, heading east
    assert east.yaw == pytest.approx(0.0, abs=1e-6)
    # Uphill to the east (nose up: negative pitch), left side (north) higher.
    assert east.pitch == pytest.approx(-math.atan(0.05), abs=1e-4)
    assert east.roll == pytest.approx(math.atan(0.02), abs=1e-4)
    # 2 m up the 4 m eastern leg, after 14 m east and a 90 deg turn of 7 m.
    north = car.pose((14 + 7 * math.pi / 2 + 2) / 5.0)
    assert north.yaw == pytest.approx(math.pi / 2, abs=1e-3)
    assert north.pitch == pytest.approx(-math.atan(0.02), abs=1e-4)
    assert north.roll == pytest.approx(-math.atan(0.05), abs=1e-4)
    assert person.pose(1.0).pitch == person.pose(1.0).roll == 0.0
    # The orientation quaternion is yaw, then pitch, then roll (ZYX).
    w, x, y, z = east.quaternion_wxyz()
    assert w * w + x * x + y * y + z * z == pytest.approx(1.0)
    forward = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
    assert forward[2] == pytest.approx(math.sin(math.atan(0.05)), abs=1e-4)


# --- ground truth -----------------------------------------------------------

def test_truth_message_follows_the_contract():
    tracks = resolve([CAR])
    message = json.loads(tt.truth_json(12.5, tracks.poses(12.5), tracks.origin))
    assert set(message) == {'stamp', 'targets'} and message['stamp'] == 12.5
    [target] = message['targets']
    assert set(target) == {'name', 'class', 'east', 'north', 'up', 'yaw'}
    assert (target['name'], target['class']) == ('car_1', 'car')
    pose = tracks.tracks[0].pose(12.5)
    # ENU about the navigation origin (the vehicle's home), not the terrain's.
    assert (target['east'], target['north'], target['up']) == pytest.approx(
        (pose.east + 20.0, pose.north + 30.0, pose.up - 2.0), abs=1e-3)
    assert target['yaw'] == pytest.approx(pose.yaw, abs=1e-4)
    assert all(isinstance(v, float) for k, v in target.items() if k not in ('name', 'class'))


def test_truth_origin_is_the_home_the_simulators_fly_from():
    """ArduPilot's EKF origin, the anchor of /navigation/odometry, is the
    home: the simulators feed it positions relative to manifest home_enu_m
    (ardupilot_json.state_from_enu reference_enu), and the builder records
    that home as the truth's origin."""
    source = (SCRIPTS / 'build_georeferenced_terrain.py').read_text()
    main = source[source.index('def main():'):]
    assert ('origin_up = terrain_collision_height(\n            np.load(find_dem(args)), '
            'args.origin_alt, args.area_km, *home) + SPAWN_HEIGHT_M') in main
    assert '"home_enu_m": [home[0], home[1], home_ground + SPAWN_HEIGHT_M]' in main
    gazebo = source[source.index('def write_gazebo_assets('):source.index('def main():')]
    assert ('ground = terrain_collision_height(dem, args.origin_alt, args.area_km, east, north)'
            in gazebo and 'return ground' in gazebo)


# --- the scenario's targets ----------------------------------------------------

def footprint_half_m(agl, fov_deg=NARROW_FOV_DEG):
    """Half the short side of the camera's ground footprint."""
    return agl * math.tan(math.radians(fov_deg) / 2) * ASPECT


# Footprints (length, width) of the baked models, metres (the car's with
# its mirrors).
MODEL_SIZE_M = {'hatchback_blue': (4.0, 2.14), 'walking_person': (0.9, 0.6),
                'casual_female': (0.6, 0.6)}


# The aircraft's horizontal speed limit (systems/aerodrone/config/tracking.env
# TRACKING_MAX_SPEED_MPS, ArduPilot's WPNAV_SPEED) and acceleration
# (arducopter.params WPNAV_ACCEL, 2 m/s^2).
MAX_SPEED_MPS, MAX_ACCEL_MPS2 = 5.0, 2.0


def footprint_corners(pose, model):
    length, width = MODEL_SIZE_M[model]
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return [(pose.east + c * dx - s * dy, pose.north + s * dx + c * dy)
            for dx in (-length / 2, length / 2) for dy in (-width / 2, width / 2)]


def scenario_tracks():
    return tt.Tracks(tt.resolve_tracks(CONFIG, lambda e, n: np.zeros_like(e) - 30.0,
                                       (*HOME, -29.8)))


def test_scenario_has_a_looping_car_and_people_near_home_around_the_hold():
    scenario = CONFIG['scenario']
    assert 30.0 <= scenario['hold_agl_m'] <= 60.0
    moving = [t for t in CONFIG['targets'] if t.get('speed_mps', 0) > 0]
    speeds = {t['class']: t['speed_mps'] for t in moving}
    assert {t['class'] for t in CONFIG['targets']} <= {'car', 'person'}  # COCO names
    assert speeds['person'] == pytest.approx(1.5, abs=0.3)  # walking
    assert [t['class'] for t in CONFIG['targets'] if not t.get('speed_mps')] == ['person']
    assert all(len(t['route_enu_m']) >= 3 for t in moving)  # loops
    hold = np.array(scenario['hold_enu_m'])
    # Whatever the drone's yaw, the camera sees a square of this half size
    # from the hold; the people stay inside it with 2 m to spare for the
    # drone's drift, the car crosses it on every lap (for 10 s or more).
    half = footprint_half_m(scenario['hold_agl_m']) - 2.0
    for track in scenario_tracks().tracks:
        period = track.length / track.speed if track.speed else 1.0
        inside = []
        for t in np.linspace(0.0, period, 400):
            pose = track.pose(t)
            assert math.dist((pose.east, pose.north), HOME) < 100.0   # near home
            corners = footprint_corners(pose, track.model)
            inside.append(np.abs(np.subtract(corners, hold)).max() <= half)
        if track.name.startswith('person'):
            assert all(inside), track.name
        else:
            assert 10.0 <= np.mean(inside) * period and not all(inside), track.name


def test_the_car_can_be_followed_within_the_aircraft_limits():
    """A fair test of the follow modes: the car drives at 60-70 % of the
    aircraft's speed limit, so the aircraft can catch up after a lag, its
    turns (13 m radius) ask the aircraft above it for about half its
    acceleration, and each straight is four slant stand-offs long (60 m at
    70 deg: 20.5 m), so a slant follower drives behind it rather than
    circling inside its loop. The first scenario's car lapped a
    26 m x 16 m circuit at the 5 m/s limit itself: the aircraft trailed it
    by 11 m (median) even with the ground-truth detector; on a 60 m x 24 m
    oval the slant follower sat inside the loop, the car crossing its view
    sideways."""
    [car] = [t for t in CONFIG['targets'] if t['class'] == 'car']
    assert 0.6 * MAX_SPEED_MPS <= car['speed_mps'] <= 0.7 * MAX_SPEED_MPS
    rows, _ = tt.resample(tt.route_polyline(car['route_enu_m'], car['corner_radius_m'],
                                             arc_step_deg=0.5), 0.2)
    # Curvature (rad per metre) over 2 m of route, and the turn's radius.
    heading = np.unwrap(rows[:, 3])
    turn = np.abs(heading[10:] - heading[:-10]) / (rows[10:, 0] - rows[:-10, 0])
    assert car['speed_mps'] ** 2 * turn.max() <= 0.55 * MAX_ACCEL_MPS2
    assert 1.0 / turn.max() >= 10.0
    # Straights long enough for the lead to settle and a slant follower
    # to stay behind the car.
    standoff = 60.0 * math.cos(math.radians(70.0))
    straight = np.concatenate(([False], turn < 1e-3, [False])).astype(int)
    edges = np.flatnonzero(np.diff(straight))
    lengths = (edges[1::2] - edges[::2]) * 0.2
    assert len(lengths) == 2 and lengths.min() >= 4.0 * standoff


def test_the_people_are_apart_from_each_other_and_the_car():
    """The walking person (the occlusion step's target) is never within
    the tracker's handover reach (4 m plus half of 2.5 s at 1.5 m/s) of the
    standing one, and the car keeps clear of both."""
    tracks = {track.name: track for track in scenario_tracks().tracks}
    walker, stander, car = tracks['person_1'], tracks['person_2'], tracks['car_1']
    still = stander.pose(0.0)
    for t in np.linspace(0.0, 2 * walker.length / walker.speed, 300):
        pose = walker.pose(t)
        assert math.dist((pose.east, pose.north), (still.east, still.north)) > 4.0 + 1.9 + 3.0
        for other in (pose, still):
            for u in np.linspace(0.0, car.length / car.speed, 120):
                drive = car.pose(u)
                assert math.dist((drive.east, drive.north), (other.east, other.north)) > 5.0


def test_every_model_is_pinned_and_public_domain():
    used = {target['model'] for target in CONFIG['targets']}
    assert used == set(CONFIG['models'])
    for model in CONFIG['models'].values():
        assert model['license'] == CC0
        assert model['fuel'].startswith('OpenRobotics/models/')
        assert isinstance(model['version'], int)
        assert len(model['sha256']) == 64 and int(model['sha256'], 16) >= 0
    notices = (REPO / 'THIRD_PARTY_NOTICES.md').read_text()
    for model in CONFIG['models'].values():
        assert model['fuel'].split('/models/')[1] in notices


# --- Fuel models ----------------------------------------------------------------

def fuel_zip(files):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w') as archive:
        for name, data in files.items():
            archive.writestr(name, data)
    return buffer.getvalue()


def test_fetch_checks_licence_and_checksum_and_then_works_offline(tmp_path):
    payload = fuel_zip({'model.sdf': '<sdf/>', 'meshes/car.obj': 'v 0 0 0\n'})
    spec = {'fuel': 'OpenRobotics/models/Hatchback blue', 'version': 5, 'license': CC0,
            'sha256': hashlib.sha256(payload).hexdigest()}
    calls = []

    def getter(url):
        calls.append(url)
        if url.endswith('.zip'):
            return payload
        return json.dumps({'license_name': CC0, 'license_url': 'https://cc0'}).encode()

    with pytest.raises(RuntimeError, match='SHA-256'):
        tm.fetch_model('car', dict(spec, sha256='0' * 64), tmp_path, getter)
    with pytest.raises(RuntimeError, match='licence'):
        tm.fetch_model('car', dict(spec, license='CC-BY-NC'), tmp_path, getter)
    assert not (tmp_path / 'car').exists()
    directory = tm.fetch_model('car', spec, tmp_path, getter)
    assert (directory / 'meshes/car.obj').read_text() == 'v 0 0 0\n'
    record = json.loads((directory / 'fuel.json').read_text())
    assert record['url'] == ('https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
                             'Hatchback%20blue/5/Hatchback%20blue.zip')
    assert (record['license'], record['sha256']) == (CC0, spec['sha256'])
    calls.clear()
    assert tm.fetch_model('car', spec, tmp_path, getter) == directory and calls == []


def write_png(path, rgba):
    import cv2
    cv2.imwrite(str(path), np.asarray(rgba, dtype=np.uint8))


def test_bake_writes_one_textured_visual_only_model_front_along_x(tmp_path):
    source = tmp_path / 'source'
    (source / 'meshes').mkdir(parents=True)
    (source / 'materials/textures').mkdir(parents=True)
    texture = np.zeros((8, 8, 4), dtype=np.uint8)
    texture[..., :3] = (200, 30, 10)  # BGR
    texture[..., 3] = 255
    texture[:2, :, 3] = 0  # a transparent strip
    write_png(source / 'materials/textures/body.png', texture)
    (source / 'meshes/car.mtl').write_text(
        'newmtl Body\nKd 1 1 1\nmap_Kd https://fuel/x/materials/textures/body.png\n'
        'newmtl Tyre\nKd 0.5 0.5 0.5\n')
    # A box 10 x 40 x 15 units, its front (longest axis) along -y, and a
    # separate tyre triangle; units are inches.
    (source / 'meshes/car.obj').write_text('\n'.join([
        'mtllib car.mtl', 'v -5 -20 0', 'v 5 -20 0', 'v 5 20 0', 'v -5 20 15',
        'vt 0 0', 'vt 1 0', 'vt 1 1', 'vt 0 1',
        'usemtl Body', 'f 1/1 2/2 3/3 4/4', 'usemtl Tyre', 'f 1/1 2/2 4/4', '']))
    spec = {'fuel': 'OpenRobotics/models/Car', 'version': 1, 'license': CC0,
            'mesh': 'meshes/car.obj', 'scale': 0.0254, 'rotate_rpy_deg': [0, 0, 90],
            'material_colors': {'^Tyre': [0.1, 0.1, 0.1]}}
    summary = tm.bake(spec, source, tmp_path / 'car', 'car')
    assert summary['triangles'] == 3 and summary['materials'] == ['Body', 'Tyre']
    mesh = np.load(tmp_path / 'car/model_mesh.npz')
    positions, uvs, triangles = mesh['positions'], mesh['uvs'], mesh['triangles']
    low, high = positions.min(axis=0), positions.max(axis=0)
    # Metres, the long axis along x, the footprint centred, standing at z 0.
    assert high - low == pytest.approx([40 * 0.0254, 10 * 0.0254, 15 * 0.0254], abs=1e-5)
    assert (low[:2] + high[:2]) == pytest.approx([0, 0], abs=1e-6) and low[2] == 0.0
    assert triangles.max() < len(positions) and np.all((0 <= uvs) & (uvs <= 1))
    # One atlas: the body's texture (its transparent strip filled) and the
    # tyre's override colour, each in its own cell.
    import cv2
    atlas = cv2.imread(str(tmp_path / 'car/car_atlas.png'))
    body_uv = uvs[triangles[:2].reshape(-1)]
    tyre_uv = uvs[triangles[2]]
    for uv, colour in ((body_uv, (200, 30, 10)), (tyre_uv, (26, 26, 26))):
        u, v = uv.mean(axis=0)
        texel = atlas[int((1 - v) * atlas.shape[0]), int(u * atlas.shape[1])]
        assert texel.tolist() == pytest.approx(list(colour), abs=2)
    # Gazebo shares materials and textures by name: both are the model's own.
    assert 'usemtl car_atlas' in (tmp_path / 'car/model.obj').read_text()
    assert (tmp_path / 'car/model.mtl').read_text().splitlines()[-1] == 'map_Kd car_atlas.png'
    sdf = ElementTree.parse(tmp_path / 'car/model.sdf').getroot()
    assert sdf.find('.//collision') is None and sdf.findtext('.//static') == 'false'
    assert sdf.findtext('.//link/gravity') == 'false'
    assert sdf.find('.//visual/geometry/mesh/uri').text == 'model://car/model.obj'
    assert CC0 in (tmp_path / 'car/ATTRIBUTION.txt').read_text()


DAE = """<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="centimeter" meter="0.01"/><up_axis>Y_UP</up_axis></asset>
<library_images><image id="skin_png"><init_from>../materials/textures/skin.png</init_from></image></library_images>
<library_effects><effect id="skin-effect"><profile_COMMON>
<newparam sid="skin-surface"><surface type="2D"><init_from>skin_png</init_from></surface></newparam>
<newparam sid="skin-sampler"><sampler2D><source>skin-surface</source></sampler2D></newparam>
<technique sid="common"><lambert><diffuse><texture texture="skin-sampler" texcoord="UVMap"/></diffuse></lambert></technique>
</profile_COMMON></effect>
<effect id="cloth-effect"><profile_COMMON><technique sid="common"><phong><diffuse><color>0.1 0.2 0.3 1</color></diffuse></phong></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="skin-material" name="Skin"><instance_effect url="#skin-effect"/></material>
<material id="cloth-material" name="Cloth"><instance_effect url="#cloth-effect"/></material></library_materials>
<library_geometries><geometry id="body-mesh"><mesh>
<source id="body-pos"><float_array id="body-pos-array" count="15">0 0 0 100 0 0 100 100 0 0 100 0 0 0 100</float_array>
<technique_common><accessor source="#body-pos-array" count="5" stride="3"/></technique_common></source>
<source id="body-uv"><float_array id="body-uv-array" count="4">0 0 1 1</float_array>
<technique_common><accessor source="#body-uv-array" count="2" stride="2"/></technique_common></source>
<vertices id="body-vertices"><input semantic="POSITION" source="#body-pos"/></vertices>
<polylist material="skin-mat" count="1"><input semantic="VERTEX" source="#body-vertices" offset="0"/>
<input semantic="TEXCOORD" source="#body-uv" offset="1" set="0"/><vcount>4</vcount><p>0 0 1 1 2 1 3 0</p></polylist>
<triangles material="cloth-mat" count="1"><input semantic="VERTEX" source="#body-vertices" offset="0"/>
<p>0 1 4</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="body"><matrix>1 0 0 50 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
<instance_geometry url="#body-mesh"><bind_material><technique_common>
<instance_material symbol="skin-mat" target="#skin-material"/><instance_material symbol="cloth-mat" target="#cloth-material"/>
</technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
"""


def test_collada_reader_applies_units_up_axis_node_transforms_and_materials(tmp_path):
    path = tmp_path / 'person.dae'
    path.write_text(DAE)
    parts, materials = tm.read_model(path)
    assert [(part.material, len(part.positions)) for part in parts] == [('Skin', 2), ('Cloth', 1)]
    # The quad is fanned into two triangles; centimetres to metres; the node
    # moves it 0.5 m along x; Y up becomes Z up (y -> z, z -> -y).
    assert np.allclose(parts[0].positions[0], [[0.5, 0, 0], [1.5, 0, 0], [1.5, 0, 1.0]])
    assert np.allclose(parts[1].positions[0][2], [0.5, -1.0, 0.0])
    assert parts[0].uvs[0].tolist() == [[0, 0], [1, 1], [1, 1]]
    assert materials['Skin'].texture == '../materials/textures/skin.png'
    assert materials['Cloth'].color == pytest.approx((0.1, 0.2, 0.3))


# --- the tracking scenario's tree ------------------------------------------------

def flat_terrain(tmp_path, ground=-30.0):
    grid = np.full((9, 9), 781.0 + ground, dtype=np.float32)
    height = tt.MeshHeight(grid_mesh(lambda e, n: ground, half=4000.0, count=9))
    return grid, height


def tracking_tree(tmp_path, ground=-30.0):
    grid, height = flat_terrain(tmp_path, ground)
    args = NS(origin_alt=781.0, area_km=8.0)
    target = builder.write_tracking_tree(grid, args, tmp_path, CONFIG['scenario'], height,
                                         tuple(HOME))
    return target, (tmp_path / 'tracking_mission.xml').read_text()


def test_tracking_tree_flies_to_the_hold_and_hovers_until_stopped(tmp_path):
    target, xml = tracking_tree(tmp_path)
    scenario = CONFIG['scenario']
    steps = list(ElementTree.fromstring(xml).find('./BehaviorTree/Sequence'))
    ids = [step.get('ID') if step.tag == 'Action' else step.tag for step in steps]
    assert ids == ['Arm', 'Offboard', 'TakeOff', 'Delay', 'GoTo', 'Delay', 'GoTo', 'Delay',
                   'KeepRunningUntilFailure']
    go_to = steps[4]
    assert steps[6].attrib == go_to.attrib
    # The hold in the navigation frame (ENU about home, whose reference is
    # 0.2 m above the ground there): hold_agl_m above the ground under it.
    east, north, up = (float(v) for v in go_to.get('pose').split(';'))
    assert (east, north) == pytest.approx(np.subtract(scenario['hold_enu_m'], HOME))
    assert up == pytest.approx(scenario['hold_agl_m'] - 0.2, abs=1e-3)
    assert target == pytest.approx((east, north, up))
    # Fixed yaw (as2 FIXED_YAW) so the footprint is the same at every hold.
    assert (go_to.get('yaw_mode'), float(go_to.get('yaw_angle'))) == (
        '2', pytest.approx(math.radians(scenario['yaw_deg'])))
    # Hover: the tree never ends and commands nothing more, so the tracker
    # can take over (a running GoTo would pre-empt it); no landing.
    hold = steps[-1]
    assert [child.tag for child in hold.iter()] == ['KeepRunningUntilFailure', 'Delay',
                                                     'AlwaysSuccess']
    assert 'Land' not in ids and 'Disarm' not in ids


def go_to_pattern(xml):
    """(GoTo, delay, same GoTo, delay) groups of a generated tree."""
    steps = [step for step in ElementTree.fromstring(xml).find('./BehaviorTree/Sequence')]
    groups = []
    for i, step in enumerate(steps):
        if step.get('ID') == 'GoTo' and (not groups or groups[-1][-1] < i - 3):
            group = steps[i:i + 4]
            groups.append([(item.tag, item.get('ID'), item.get('delay_msec')) for item in group]
                          + [steps[i + 2].attrib == step.attrib, i])
    return groups


def test_tracking_tree_flies_its_waypoint_like_the_wilderness_tree(tmp_path):
    """build_georeferenced_terrain.py sends each wilderness waypoint's GoTo
    twice, 2 s apart (as2 1.1.3 can end a new GoTo at once on the previous
    goal's distance): the tracking tree's hold is flown the same way."""
    _, tracking = tracking_tree(tmp_path)
    mission = tmp_path / 'mission.json'
    mission.write_text(json.dumps(MISSION))
    grid, _ = flat_terrain(tmp_path)
    builder.write_behavior_tree(grid, NS(mission=str(mission), origin_alt=781.0, area_km=8.0,
                                         cruise_agl=250.0), tmp_path)
    wilderness = go_to_pattern((tmp_path / 'wilderness_mission.xml').read_text())
    [hold] = go_to_pattern(tracking)
    shape = [group[:4] for group in wilderness]
    assert all(entry[:3] == shape[0][:3] for entry in shape)
    assert hold[:3] == shape[0][:3] and hold[3][:2] == ('Delay', None)
    assert int(hold[3][2]) >= 2000 and hold[4] and all(group[4] for group in wilderness)


def test_flight_ready_waits_for_every_server_the_tracking_tree_calls(tmp_path):
    """flight_ready (wait_for_mavros_navigation.py) gates the tree it is
    given on the servers of its nodes: the tracking tree uses no node it
    does not know."""
    import ast
    from test_aerial_mission_start import BT_BUILTINS
    source = (REPO / 'applications/aerostack2/scripts/wait_for_mavros_navigation.py').read_text()
    known = set()
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and node.targets[0].id in ('BEHAVIOR_ACTIONS',
                                                                    'BEHAVIOR_SERVICES'):
            known |= set(ast.literal_eval(node.value))
    _, xml = tracking_tree(tmp_path)
    tree = ElementTree.fromstring(xml)
    used = {element.get('ID') if element.tag == 'Action' else element.tag
            for element in tree.find('BehaviorTree').iter()} - {'BehaviorTree'}
    assert used - (BT_BUILTINS | {'KeepRunningUntilFailure'}) == {
        'Arm', 'Offboard', 'TakeOff', 'GoTo'}
    assert {'Arm', 'Offboard', 'TakeOff', 'GoTo'} <= known


# --- the builder's tracking assets --------------------------------------------------

def test_builder_bakes_the_models_resolves_the_routes_and_places_them_in_the_world(tmp_path):
    cache = tmp_path / 'cache/walker'
    (cache / 'meshes').mkdir(parents=True)
    (cache / 'meshes/walker.obj').write_text(
        'v -0.3 0 0\nv 0.3 0 0\nv 0 0.2 1.8\nvt 0 0\nvt 1 0\nvt 0 1\nf 1/1 2/2 3/3\n')
    (cache / 'fuel.json').write_text(json.dumps({'url': 'u', 'license_url': 'l'}))
    config = {'sample_spacing_m': 0.5,
              'models': {'walker': {'fuel': 'OpenRobotics/models/Walker', 'version': 1,
                                    'license': CC0, 'sha256': '0' * 64,
                                    'mesh': 'meshes/walker.obj'}},
              'targets': [{'name': 'person_1', 'class': 'person', 'model': 'walker',
                           'speed_mps': 1.5, 'corner_radius_m': 1.0,
                           'route_enu_m': [[0, 0], [10, 0], [10, 6], [0, 6]]}],
              'scenario': CONFIG['scenario']}
    (tmp_path / 'targets.json').write_text(json.dumps(config))
    out = tmp_path / 'assets'
    out.mkdir()
    np.savez_compressed(out / 'terrain_mesh.npz',
                        vertices=grid_mesh(plane).astype(np.float32))
    args = NS(targets=str(tmp_path / 'targets.json'), target_cache=str(tmp_path / 'cache'))
    loaded, height, includes = builder.write_tracking_targets(args, out, (-7.0, -8.0, 1.5))
    assert loaded == config and height(3.0, 4.0) == pytest.approx(plane(3.0, 4.0), abs=1e-4)
    document = json.loads((out / 'tracking_targets.json').read_text())
    assert document['navigation_origin_enu_m'] == [-7.0, -8.0, 1.5]
    tracks = tt.Tracks.load(out / 'tracking_targets.json')
    [track] = tracks.tracks
    for kind in ('mesh', 'mesh_npz', 'texture'):
        assert tracks.model_file(track, kind).is_file()
    assert tracks.model_file(track, 'mesh') == out / 'models/walker/model.obj'
    start = tracks.poses(0.0)[0]
    [include] = ElementTree.fromstring(f'<w>{includes}</w>')
    assert include.findtext('uri') == 'model://walker' and include.findtext('name') == 'person_1'
    assert [float(v) for v in include.findtext('pose').split()] == pytest.approx(
        [start.east, start.north, start.up, 0, 0, start.yaw], abs=1e-3)
    with pytest.raises(FileNotFoundError, match='target_assets'):
        builder.write_tracking_targets(NS(targets=args.targets,
                                          target_cache=str(tmp_path / 'empty')), out, (0, 0, 0))

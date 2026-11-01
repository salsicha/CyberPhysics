"""tools/wildnav_benchmark.py: its stills are the Gazebo stack's synthetic
downward camera's (applications/gazebo/scripts/aerodrone_sensor_sim.py),
pixel for pixel, and its profiles are the stacks' settings."""
import math
import re
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
import yaml

import synthetic_world as sw
from conftest import REPO, install_ros_stubs, load_script

SITE = (37.9234, -122.5967)


@pytest.fixture(scope='module')
def benchmark():
    return load_script(REPO / 'tools/wildnav_benchmark.py', 'wildnav_benchmark_review')


@pytest.fixture
def sensor_sim():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.executors', 'geographic_msgs', 'geographic_msgs.msg'):
            sys.modules[name] = MagicMock()
        yield load_script(REPO / 'applications/gazebo/scripts/aerodrone_sensor_sim.py',
                          'aerodrone_sensor_sim_benchmark_review')


@pytest.mark.parametrize('east,north,yaw_deg', [
    (120.0, -80.0, 0.0), (-310.0, 45.0, 37.0), (12.0, 400.0, -120.0),
    # Near a tile edge, where the simulator's crop is clamped into the tile.
    (0.0, 0.0, 90.0)])
def test_stills_are_the_simulators(benchmark, sensor_sim, tmp_path, east, north,
                                   yaw_deg):
    lat, lon = sw.local_to_latlon(east, north, *SITE)
    tile_x, tile_y, pixel_x, pixel_y = benchmark.tile_position(lat, lon, 17)
    rng = np.random.default_rng(3)
    tile = rng.integers(0, 255, (256, 256), dtype=np.uint8)
    cv2.imwrite(str(tmp_path / f'17_{tile_x}_{tile_y}.jpg'), tile)

    sim = sensor_sim.AerodroneSensorSim.__new__(sensor_sim.AerodroneSensorSim)
    sim.synthetic_width, sim.synthetic_height = benchmark.GAZEBO_SIZE
    sim.wildnav_zoom = 17
    sim.satellite_cache_dir = str(tmp_path)
    sim.origin_lat, sim.origin_lon = SITE
    yaw = math.radians(yaw_deg)
    sim.latest_odom = NS(pose=NS(pose=NS(
        position=NS(x=east, y=north, z=250.0),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
    expected = sim._synthetic_satellite_image()

    # The tile as the simulator reads it (its JPEG, in colour).
    read = cv2.imread(str(tmp_path / f'17_{tile_x}_{tile_y}.jpg'), cv2.IMREAD_COLOR)
    still = benchmark.gazebo_frame(read, pixel_x, pixel_y, yaw)
    assert still.shape == expected.shape == (480, 640, 3)
    np.testing.assert_array_equal(still, expected)


def stack_args(path, service):
    command = yaml.safe_load(path.read_text())['services'][service]['command']
    return {key: value for key, value in re.findall(r'(\w+):=(\S+)', command)}


def fallback(value):
    match = re.fullmatch(r'\$\{\w+:-(.*)\}', value)
    return match.group(1) if match else value


def test_lite_profile_is_the_companion_computers(benchmark):
    aircraft = stack_args(REPO / 'compositions/blueos_nav_core.yaml', 'wildnav')
    hil = stack_args(REPO / 'tools/aerodrone_hil_pi.yaml', 'wildnav')
    for key, value in benchmark.LITE.items():
        assert float(fallback(aircraft[key])) == value
        assert float(fallback(hil[key])) == value
    assert benchmark.PROFILES['aircraft'] == {
        'zoom': int(fallback(aircraft['zoom'])),
        'search_radius_m': float(fallback(aircraft['search_radius_m'])),
        'max_search_radius_m': float(fallback(aircraft['max_search_radius_m'])),
        'max_tiles': int(fallback(aircraft['max_tiles']))}


def test_site_profile_is_the_simulated_sites(benchmark):
    env = dict(line.split('=', 1) for line in (
        REPO / 'applications/aerostack2/config/mount_tamalpais.env').read_text().splitlines()
        if line and not line.startswith('#'))
    site = benchmark.PROFILES['site']
    assert site == {'zoom': int(env['WILDNAV_ZOOM']),
                    'search_radius_m': float(env['WILDNAV_SEARCH_RADIUS_M']),
                    'max_search_radius_m': float(env['WILDNAV_MAX_SEARCH_RADIUS_M']),
                    'max_tiles': int(env['WILDNAV_MAX_TILES'])}
    assert benchmark.PROFILES['site_lite'] == {**site, **benchmark.LITE}
    assert benchmark.SITE == (float(env['INITIAL_LAT']), float(env['INITIAL_LON']))


def test_profile_arguments(benchmark):
    name, params = benchmark.parse_profile(
        'quick=base:site,match_stop_confidence:0.6,opencv_threads:1,camera_stabilized:true')
    assert name == 'quick'
    assert params == {**benchmark.PROFILES['site'], 'match_stop_confidence': 0.6,
                      'opencv_threads': 1, 'camera_stabilized': True}
    assert benchmark.parse_profile('site_lite') == ('site_lite', benchmark.PROFILES['site_lite'])

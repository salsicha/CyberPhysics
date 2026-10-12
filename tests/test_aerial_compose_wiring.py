"""Aerial stacks: shared EKF origin, DemNav/WildNav prior/origin parameters,
camera mounts, and hardware-safe ArduPilot parameter layering."""
import re

import pytest
import yaml

from conftest import REPO

COMPOSITIONS = REPO / 'compositions'
SIM_MOUNTS = {'aerostack2_sim.yaml': '0.0', 'aerostack2_isaac.yaml': '90.0'}


def services(name):
    return yaml.safe_load((COMPOSITIONS / name).read_text())['services']


@pytest.mark.parametrize('name', sorted(SIM_MOUNTS))
def test_aerodrone_sims_share_the_ekf_origin_and_fused_prior(name):
    stack = services(name)
    assert 'ardupilot_navigation_bridge.py' in stack['navigation_bridge']['command']
    for service in ('demnav', 'wildnav'):
        command = stack[service]['command']
        assert 'origin_topic:=/navigation/origin' in command
        assert 'prior_fix_topic:=/navigation/fix' in command
        assert 'origin_alt:=' not in command
    demnav = stack['demnav']['command']
    assert f'camera_mount_yaw_deg:={SIM_MOUNTS[name]}' in demnav
    assert 'gps_altitude_datum:=ellipsoid' in demnav
    assert 'GPS_INPUT_CORRECTIONS' in str(stack['navigator_sitl']['environment'])


def test_every_stack_running_demnav_or_wildnav_wires_the_new_parameters():
    for path in sorted(COMPOSITIONS.glob('*.yaml')):
        text = path.read_text()
        for launch in ('demnav.launch.py', 'wildnav.launch.py'):
            for match in re.finditer(launch, text):
                block = text[match.end():match.end() + 3000].split('\n\n')[0]
                assert 'origin_topic:=' in block, (path.name, launch)
                assert 'prior_fix_topic:=' in block, (path.name, launch)
                assert 'max_search_radius_m:=' in block, (path.name, launch)
                assert 'origin_alt:=' not in block, (path.name, launch)
                if launch == 'demnav.launch.py':
                    assert 'camera_mount_yaw_deg:=' in block, path.name
                    assert 'rangefinder_topic:=' in block, path.name
                    assert 'gps_altitude_datum:=' in block, path.name


def test_hardware_compositions_never_load_sitl_relaxations():
    for path in sorted(COMPOSITIONS.glob('*_hardware.yaml')) + [
            COMPOSITIONS / 'blueos_nav_core.yaml']:
        assert '_sitl.params' not in path.read_text(), path.name


@pytest.mark.parametrize('name,firmware', [('boat_sim.yaml', 'ardurover'),
                                           ('submarine_sim.yaml', 'ardusub')])
def test_marine_sims_layer_sitl_relaxations_after_the_baseline(name, firmware):
    sitl = services(name)['navigator_sitl']['environment']
    assert sitl['ARDUPILOT_PARAM_FILE'].endswith(f'/{firmware}.params')
    assert sitl['ARDUPILOT_SITL_PARAM_FILE'].endswith(f'/{firmware}_sitl.params')
    # Arming talks to MAVROS over shared-memory DDS: same IPC namespace.
    assert services(name)['navigator_arm']['ipc'] == 'host'


def test_copter_sitl_layers_sitl_relaxations_by_default():
    script = (REPO / 'applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh').read_text()
    order = [script.index(name) for name in (
        'base_params" "$sitl_params" "$gps_input_params" "$backend_params')]
    assert order
    assert '${config_dir}/arducopter_sitl.params' in script
    assert 'GPS_INPUT_CORRECTIONS:-true' in script


def _env_value(path, key):
    for line in path.read_text().splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return None


@pytest.mark.parametrize('name', sorted(SIM_MOUNTS) + ['blueos_nav_core.yaml'])
def test_gps_input_corrections_default_on_for_every_aerodrone_stack(name):
    """The SITL loads arducopter_gps_input.params and the bridge forwards
    with the same flag, on unless GPS_INPUT_CORRECTIONS=false."""
    text = (COMPOSITIONS / name).read_text()
    assert 'GPS_INPUT_CORRECTIONS:-false' not in text
    bridge = services(name)['navigation_bridge']['command']
    assert 'gps_input_enabled:=${GPS_INPUT_CORRECTIONS:-true}' in bridge
    if name != 'blueos_nav_core.yaml':
        sitl = services(name)['navigator_sitl']['environment']
        assert sitl['GPS_INPUT_CORRECTIONS'] == '${GPS_INPUT_CORRECTIONS:-true}'
    assert _env_value(REPO / 'systems/aerodrone/config/topics.env',
                      'GPS_INPUT_CORRECTIONS') == 'true'


def test_gps_input_corrections_stay_off_for_arduplane():
    """Designed and validated for ArduCopter only."""
    bridge = services('airplane_sim.yaml')['airplane_navigation_bridge']['command']
    assert 'gps_input_enabled:=false' in bridge
    assert _env_value(REPO / 'systems/airplane/config/blueos.env',
                      'GPS_INPUT_CORRECTIONS') == 'false'


def env_value(name):
    env = (REPO / 'applications/aerostack2/config/mount_tamalpais.env').read_text()
    return float(re.search(rf'^{name}=(.*)$', env, re.MULTILINE).group(1))


@pytest.mark.parametrize('name', sorted(SIM_MOUNTS))
def test_mission_cruises_at_the_configured_altitude(name):
    command = services(name)['terrain_assets']['command']
    assert '--cruise-agl ${CRUISE_AGL_M:-' in command
    # DemNav needs the footprint of a high cruise to find a unique match.
    assert env_value('CRUISE_AGL_M') >= 200.0


def test_every_simulated_camera_sees_the_ground_from_the_cruise():
    """Depth and colour far planes leave room for the cruise height plus
    the terrain's relief across the footprint (twice the cruise height)."""
    needed = 2.0 * env_value('CRUISE_AGL_M')
    sources = {
        'applications/isaac/scripts/aerodrone_navsim.py':
            r'GetClippingRangeAttr\(\)\.Set\(Gf\.Vec2f\([0-9.]+, ([0-9.]+)\)\)',
        'applications/gazebo/models/rgbd_camera.sdf.jinja': r'<far>([0-9.]+)</far>',
        'applications/gazebo/models/hd_camera.sdf.jinja': r'<far>([0-9.]+)</far>',
    }
    for path, pattern in sources.items():
        planes = [float(value) for value in re.findall(pattern, (REPO / path).read_text())]
        assert planes and min(planes) >= needed, (path, planes)


def test_cruise_footprint_stays_inside_the_imagery_and_dem():
    """Every en-route waypoint's camera footprint at the cruise height lies
    inside the seeded orthophoto (WILDNAV_SEARCH_RADIUS_M about the origin)
    and DemNav's widest search inside the DEM (AREA_KM square)."""
    import json
    import math
    cruise = env_value('CRUISE_AGL_M')
    half_fov = math.radians(env_value('FOV_DEG') / 2.0)
    # Corner of a 4:3 frame from a nadir camera.
    footprint = cruise * math.tan(half_fov) * math.hypot(1.0, 0.75)
    mission = json.loads(
        (REPO / 'applications/aerostack2/missions/wilderness_missions.json').read_text())
    farthest = max(math.hypot(*waypoint['position_enu_m'][:2])
                   for entry in mission['missions'] for phase in entry['phases']
                   for waypoint in phase['waypoints'])
    assert farthest + footprint <= env_value('WILDNAV_SEARCH_RADIUS_M')
    assert farthest + footprint + env_value('MAX_SEARCH_RADIUS_M') <= (
        500.0 * env_value('AREA_KM'))

"""Aerial stacks: shared EKF origin, DemNav/WildNav prior/origin parameters,
camera mounts, and hardware-safe ArduPilot parameter layering."""
import re

import pytest
import yaml

from conftest import REPO

COMPOSITIONS = REPO / 'compositions'
SIM_MOUNTS = {'aerostack2_sim.yaml': '0.0', 'aerostack2_genesis.yaml': '0.0',
              'aerostack2_isaac.yaml': '90.0'}


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
    assert 'GPS_INPUT_CORRECTIONS:-false' in script

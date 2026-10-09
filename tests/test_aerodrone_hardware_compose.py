"""Check resolved commands, not only Compose syntax or YAML source strings."""
import json
import os
import shutil
import subprocess

import pytest

from conftest import REPO


@pytest.mark.parametrize('namespace', [None, 'field_drone'])
def test_hardware_commands_use_consistent_vehicle_identity(namespace):
    if not shutil.which('docker'):
        pytest.skip('Docker Compose is not installed')
    version = subprocess.run(['docker', 'compose', 'version'], capture_output=True)
    if version.returncode:
        pytest.skip('Docker Compose is not installed')
    env = dict(os.environ)
    for key in ('DRONE_NAMESPACE', 'VEHICLE_NAMESPACE', 'COMPOSE_FILE', 'COMPOSE_ENV_FILES'):
        env.pop(key, None)
    if namespace:
        env.update(DRONE_NAMESPACE=namespace, VEHICLE_NAMESPACE=namespace)
    rendered = subprocess.run(
        ['docker', 'compose', '-f', str(REPO / 'compositions/aerodrone_hardware.yaml'),
         'config', '--format', 'json'],
        env=env, check=True, capture_output=True, text=True,
    )
    services = json.loads(rendered.stdout)['services']
    vehicle = namespace or 'aerodrone'
    assert len(services) == 10
    assert all(service.get('image') for service in services.values())
    assert f'namespace:={vehicle}' in services['aerostack']['command']
    assert f'drone_id:={vehicle}' in services['behavior_tree']['command']
    for name, odom_arg in [('demnav', 'odom_topic'), ('wildnav', 'raw_odom_topic')]:
        assert f'gps_topic:=/{vehicle}/sensor_measurements/gps' in services[name]['command']
        assert f'{odom_arg}:=/{vehicle}/sensor_measurements/odom' in services[name]['command']
    assert 'depth_topic:=/oak1/relative_depth' in services['demnav']['command']
    assert 'image_topic:=/oak1/image_highres' in services['wildnav']['command']
    # The EKF origin anchors every local navigation output; DemNav/WildNav
    # search around the fused fix and DemNav uses the real camera mount.
    assert 'ardupilot_navigation_bridge.py' in ' '.join(services['navigation_bridge']['command'])
    # GPS_INPUT corrections are on by default (the bridge still only forwards
    # once the flight controller reports arducopter_gps_input.params).
    assert 'gps_input_enabled:=true' in services['navigation_bridge']['command']
    for name in ('demnav', 'wildnav'):
        command = services[name]['command']
        assert 'origin_topic:=/navigation/origin' in command
        assert 'prior_fix_topic:=/navigation/fix' in command
        assert not any(arg.startswith('origin_alt:=') for arg in command)
    assert 'camera_mount_yaw_deg:=0.0' in services['demnav']['command']
    assert 'rangefinder_topic:=/aerodrone/rangefinder' in services['demnav']['command']
    # The downward Benewake rangefinder feeds DemNav's height above ground.
    rangefinder = services['rangefinder']
    assert rangefinder['image'].startswith('salsicha/cyberphysics:rangefinder-')
    assert rangefinder['devices'][0]['target'] == '/dev/rangefinder'
    assert 'port:=/dev/rangefinder' in rangefinder['command']
    assert 'topic:=/aerodrone/rangefinder' in rangefinder['command']
    assert 'model:=tfmini_s' in rangefinder['command']
    assert 'gps_altitude_datum:=ellipsoid' in services['demnav']['command']


def test_airplane_hardware_uses_its_namespaced_navigation_topics():
    if not shutil.which('docker'):
        pytest.skip('Docker Compose is not installed')
    if subprocess.run(['docker', 'compose', 'version'], capture_output=True).returncode:
        pytest.skip('Docker Compose is not installed')
    env = {key: value for key, value in os.environ.items()
           if key not in ('COMPOSE_FILE', 'COMPOSE_ENV_FILES')}
    rendered = subprocess.run(
        ['docker', 'compose', '-f', str(REPO / 'compositions/airplane_hardware.yaml'),
         'config', '--format', 'json'],
        env=env, check=True, capture_output=True, text=True)
    services = json.loads(rendered.stdout)['services']
    demnav = services['demnav']['command']
    assert 'prior_fix_topic:=/airplane/navigation/fix' in services['wildnav']['command']
    assert 'prior_fix_topic:=/airplane/navigation/fix' in demnav
    # No downward rangefinder: the YAML empty string disables it.
    assert "rangefinder_topic:=''" in demnav
    bridge = services['navigation_bridge']['command']
    assert 'correction_topic:=/airplane/navigation/correction_fix' in bridge
    # GPS_INPUT corrections are not validated on ArduPlane.
    assert 'gps_input_enabled:=false' in bridge


@pytest.mark.parametrize('value', [None, 'false'])
def test_aerodrone_hardware_gps_input_corrections_can_be_turned_off(value):
    if not shutil.which('docker'):
        pytest.skip('Docker Compose is not installed')
    if subprocess.run(['docker', 'compose', 'version'], capture_output=True).returncode:
        pytest.skip('Docker Compose is not installed')
    env = {key: item for key, item in os.environ.items()
           if key not in ('COMPOSE_FILE', 'COMPOSE_ENV_FILES', 'GPS_INPUT_CORRECTIONS')}
    if value:
        env['GPS_INPUT_CORRECTIONS'] = value
    rendered = subprocess.run(
        ['docker', 'compose', '-f', str(REPO / 'compositions/aerodrone_hardware.yaml'),
         'config', '--format', 'json'],
        env=env, check=True, capture_output=True, text=True)
    bridge = json.loads(rendered.stdout)['services']['navigation_bridge']['command']
    assert f'gps_input_enabled:={value or "true"}' in bridge

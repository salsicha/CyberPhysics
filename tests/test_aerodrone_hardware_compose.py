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
    assert len(services) == 8
    assert all(service.get('image') for service in services.values())
    assert f'namespace:={vehicle}' in services['aerostack']['command']
    assert f'drone_id:={vehicle}' in services['behavior_tree']['command']
    for name, odom_arg in [('demnav', 'odom_topic'), ('wildnav', 'raw_odom_topic')]:
        assert f'gps_topic:=/{vehicle}/sensor_measurements/gps' in services[name]['command']
        assert f'{odom_arg}:=/{vehicle}/sensor_measurements/odom' in services[name]['command']
    assert 'depth_topic:=/oak1/relative_depth' in services['demnav']['command']
    assert 'image_topic:=/oak1/image_highres' in services['wildnav']['command']

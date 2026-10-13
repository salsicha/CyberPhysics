"""Resolve real Compose includes/extends to catch a mixed ROS middleware graph."""
import json
import os
import shutil
import subprocess

import pytest
from conftest import REPO


@pytest.mark.parametrize('stack', [
    'aerodrone_hardware', 'aerostack2_sim', 'aerostack2_isaac',
    'boat_hardware', 'submarine_hardware', 'racecar_turret_hardware',
    'racecar_turret_isaac', 'turret_hardware', 'turret_sim', 'turret_uno_q',
    'so101_smolvla_isaac', 'so101_groot_isaac', 'so101_gazebo',
    'spaceros', 'lunar_rover_sim', 'lunar_rover_hardware',
])
@pytest.mark.parametrize('rmw', [None, 'rmw_fastrtps_cpp'])
def test_connected_graph_selects_one_backend_with_explicit_fallback(stack, rmw):
    if not shutil.which('docker'):
        pytest.skip('Docker Compose not installed')
    env = {k: v for k, v in os.environ.items()
           if k not in ('RMW_IMPLEMENTATION', 'ZENOH_CONFIG_OVERRIDE')}
    if rmw:
        env['RMW_IMPLEMENTATION'] = rmw
    result = subprocess.run([
        'docker', 'compose', '-f', str(REPO / 'compositions' / f'{stack}.yaml'),
        '--profile', '*', 'config', '--format', 'json',
    ], env=env, text=True, capture_output=True, check=True)
    checked = []
    for name, service in json.loads(result.stdout)['services'].items():
        values = service.get('environment', {})
        if 'ROS_DOMAIN_ID' not in values:
            continue
        assert values['RMW_IMPLEMENTATION'] == (rmw or 'rmw_zenoh_cpp'), name
        assert 'scouting/multicast/enabled=true' in values['ZENOH_CONFIG_OVERRIDE'], name
        checked.append(name)
    assert checked, stack


def test_explicit_remote_endpoint_reaches_included_flight_nodes():
    if not shutil.which('docker'):
        pytest.skip('Docker Compose not installed')
    config = 'mode="client";connect/endpoints=["tcp/192.0.2.1:7448"];scouting/multicast/enabled=false'
    result = subprocess.run([
        'docker', 'compose', '-f', str(REPO / 'compositions/aerodrone_hardware.yaml'),
        '--profile', '*', 'config', '--format', 'json',
    ], env={**os.environ, 'ZENOH_CONFIG_OVERRIDE': config},
        text=True, capture_output=True, check=True)
    for name, service in json.loads(result.stdout)['services'].items():
        values = service.get('environment', {})
        if 'ROS_DOMAIN_ID' in values:
            assert values['ZENOH_CONFIG_OVERRIDE'] == config, name


def test_embedded_default_is_native_zenoh_and_xrce_is_explicit_legacy():
    import yaml
    services = yaml.safe_load((REPO / 'compositions/arduino.yaml').read_text())['services']
    assert 'uros' not in services
    for service in services.values():
        assert service['environment']['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    serial = services['serial_router']
    assert serial['profiles'] == ['usb']
    assert 'serial//dev/zenoh_mcu#baudrate=115200' in serial['environment']['ZENOH_CONFIG_OVERRIDE']
    assert serial['devices'] == ['${ZENOH_SERIAL_DEVICE:-/dev/ttyACM0}:/dev/zenoh_mcu']
    assert all('privileged' not in service for service in services.values())
    services = yaml.safe_load((REPO / 'compositions/arduino_xrce.yaml').read_text())['services']
    for name in ('uros', 'ros_sub'):
        assert services[name]['environment']['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'


def test_env_default_checker_understands_self_overrides(tmp_path, monkeypatch):
    import importlib.util
    spec = importlib.util.spec_from_file_location('middleware_env_defaults', REPO / 'tools/check_compose_env_defaults.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = tmp_path / 'middleware.env'
    source.write_text('RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}\nZENOH_ROUTER_CHECK_ATTEMPTS=${ZENOH_ROUTER_CHECK_ATTEMPTS:--1}\nLITERAL=42\n')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    assert module.parse_env_file(source) == {
        'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp', 'ZENOH_ROUTER_CHECK_ATTEMPTS': '-1', 'LITERAL': '42'}

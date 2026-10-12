"""The BlueOS vehicle stacks run on the host network beside BlueOS itself, so
the ports their services listen on must not be ones BlueOS already holds,
their ROS containers must share the host's IPC (Fast DDS shared memory),
and MAVROS must reach the autopilot through BlueOS's MAVLink router.

rosbridge's default port, 9090, is BlueOS's network manager (Cable Guy) on
0.0.0.0:9090: on a BlueOS 1.4.2 Raspberry Pi 5 rosbridge only logged
"Unable to start server: [Errno 98] Address already in use" every 5 s and
the map page could never connect. The stacks run it on 9092, and the page
connects there.
"""
import json
import os
import re
import shutil
import subprocess

import pytest
import yaml

from conftest import REPO

COMPOSITIONS = REPO / 'compositions'
# TCP ports BlueOS 1.4.2 (core, bootstrap, Cockpit and BlueOS Cloud
# extensions) listened on, measured with `ss -ltn` on the Pi 5 Navigator
# vehicle (SSH and Docker-proxied extension ports left out).
BLUEOS_TCP_PORTS = {80, 81, 2748, 2770, 5201, 5777, 6020, 6021, 6030, 6040, 7000,
                    7117, 7447, 7777, 8000, 8081, 8088, 9000, 9090, 9100, 9101,
                    9110, 9111, 9120, 9134, 27353}
BLUEOS_STACKS = ['blueos_nav_core.yaml', 'blueos_aerostack.yaml', 'aerodrone_hardware.yaml',
                 'airplane_hardware.yaml', 'boat_hardware.yaml', 'submarine_hardware.yaml']
ROSBRIDGE_PORT = 9092


def services(name):
    return yaml.safe_load((COMPOSITIONS / name).read_text()).get('services') or {}


def command(service):
    value = service.get('command', '')
    return value if isinstance(value, str) else ' '.join(value)


def listening_ports(service):
    """TCP ports a service's command listens on: rosbridge and http.server."""
    text = command(service)
    ports = []
    if 'rosbridge_websocket_launch.xml' in text:
        match = re.search(r'\bport:=(\d+)', text)
        ports.append(int(match.group(1)) if match else 9090)
    match = re.search(r'-m http\.server (\d+)', text)
    if match:
        ports.append(int(match.group(1)))
    return ports


@pytest.mark.parametrize('name', BLUEOS_STACKS)
def test_blueos_stack_services_avoid_blueos_ports(name):
    for service_name, service in services(name).items():
        if service.get('network_mode') != 'host':
            continue
        for port in listening_ports(service):
            assert port not in BLUEOS_TCP_PORTS, f'{name}: {service_name} listens on {port}'


@pytest.mark.parametrize('name', ['blueos_nav_core.yaml', 'boat_hardware.yaml',
                                  'submarine_hardware.yaml'])
def test_blueos_rosbridge_listens_on_the_shared_port(name):
    rosbridges = [s for s in services(name).values()
                  if 'rosbridge_websocket_launch.xml' in command(s)]
    assert rosbridges
    for service in rosbridges:
        assert listening_ports(service) == [ROSBRIDGE_PORT]


def test_the_web_page_and_every_aerodrone_rosbridge_use_one_port():
    """The page connects to its host's rosbridge port: the aircraft's
    (blueos_nav_core.yaml) and the simulators' (aerodrone_tracking_sim.yaml)
    must both listen there."""
    app = (REPO / 'applications/aerostack2/src/as2_platform_blueos/web/app.js').read_text()
    assert re.findall(r"hostname \|\| 'localhost'\}:(\d+)`", app) == [str(ROSBRIDGE_PORT)]
    sim = services('aerodrone_tracking_sim.yaml')['tracking_rosbridge']
    assert listening_ports(sim) == [ROSBRIDGE_PORT]
    assert listening_ports(services('blueos_nav_core.yaml')['rosbridge']) == [ROSBRIDGE_PORT]
    script = (REPO / 'tools/tracking_ui_test.sh').read_text()
    assert f'rosbridge_websocket_launch.xml port:={ROSBRIDGE_PORT}' in script


@pytest.mark.parametrize('name', BLUEOS_STACKS)
def test_blueos_ros_services_share_host_ipc(name):
    """On the host network Fast DDS (Humble) sends to peers on the same host
    through shared memory: a ROS container with its own IPC namespace and
    /dev/shm discovers every topic but receives none of their messages.
    The stacks' rosbridges had neither, so the web page saw no data even on
    a free port (measured on the Pi 5: `ros2 topic echo /mavros/state`
    prints nothing from such a container and the message with --ipc host)."""
    for service_name, service in services(name).items():
        if service.get('network_mode') != 'host' or 'http.server' in command(service):
            continue
        assert service.get('ipc') == 'host', f'{name}: {service_name} has no ipc: host'


# MAVROS's link to the autopilot -------------------------------------------

BLUEOS_ROUTER = 'tcp://127.0.0.1:5777'
# (composition, the --env-file its README runs it with, MAVROS service)
MAVROS_STACKS = [
    ('blueos_nav_core.yaml', None, 'mavros'),
    ('blueos_aerostack.yaml', None, 'mavros'),
    ('aerodrone_hardware.yaml', None, 'mavros'),
    ('airplane_hardware.yaml', None, 'mavros'),
    ('boat_hardware.yaml', 'systems/boat/config/blueos.env', 'boat_mavros'),
    ('submarine_hardware.yaml', 'systems/submarine/config/blueos.env', 'submarine_mavros'),
]


def rendered_fcu_url(name, env_file, service, fcu_url=None):
    if not shutil.which('docker') or subprocess.run(
            ['docker', 'compose', 'version'], capture_output=True).returncode:
        pytest.skip('Docker Compose is not installed')
    env = {key: value for key, value in os.environ.items()
           if key not in ('COMPOSE_FILE', 'COMPOSE_ENV_FILES', 'BLUEOS_FCU_URL')}
    if fcu_url:
        env['BLUEOS_FCU_URL'] = fcu_url
    args = ['docker', 'compose']
    if env_file:
        args += ['--env-file', str(REPO / env_file)]
    args += ['-f', str(COMPOSITIONS / name), 'config', '--format', 'json']
    rendered = subprocess.run(args, env=env, check=True, capture_output=True, text=True)
    command = json.loads(rendered.stdout)['services'][service]['command']
    [url] = [arg.split(':=', 1)[1] for arg in command if arg.startswith('fcu_url:=')]
    return url


@pytest.mark.parametrize('name,env_file,service', MAVROS_STACKS,
                         ids=[stack[0] for stack in MAVROS_STACKS])
def test_mavros_uses_blueos_router_tcp_link_by_default(name, env_file, service):
    """Stock BlueOS sends nothing to the Pi's own UDP 14550, the old default:
    MAVROS stayed disconnected. Its router's TCP server (the protected
    "Internal Link", 127.0.0.1:5777) needs no BlueOS change; on the Pi 5
    MAVROS connected there as 1:191 next to mavlink2rest (1:194)."""
    assert rendered_fcu_url(name, env_file, service) == BLUEOS_ROUTER


@pytest.mark.parametrize('name,env_file,service', MAVROS_STACKS,
                         ids=[stack[0] for stack in MAVROS_STACKS])
def test_mavros_link_can_be_overridden(name, env_file, service):
    url = 'udp://0.0.0.0:14550@'
    assert rendered_fcu_url(name, env_file, service, fcu_url=url) == url


def test_the_vehicle_env_files_use_the_router_link():
    for name in ('airplane', 'boat', 'submarine'):
        text = (REPO / f'systems/{name}/config/blueos.env').read_text()
        assert f'BLUEOS_FCU_URL={BLUEOS_ROUTER}\n' in text, name


@pytest.mark.parametrize('name', ['blueos_nav_core.yaml', 'boat_hardware.yaml', 'submarine_hardware.yaml'])
def test_hardware_mavros_uses_required_plugin_allowlist(name):
    nodes = [s for s in services(name).values() if 'blueos_mavros.launch.xml' in command(s)]
    assert len(nodes) == 1
    assert '../applications/aerostack2/config:/workspace/mavros_config:ro' in nodes[0]['volumes']
    config = yaml.safe_load((REPO / 'applications/aerostack2/config/mavros_plugins.yaml').read_text())
    plugins = set(config['/**']['ros__parameters']['plugin_allowlist'])
    assert {'command', 'sys_status', 'sys_time', 'param', 'imu', 'global_position',
            'local_position', 'gps_status', 'gps_input', 'setpoint_velocity',
            'setpoint_position', 'setpoint_raw', 'vfr_hud', 'rc_io', 'waypoint'} <= plugins
    assert len(plugins) < 25


@pytest.mark.parametrize('name', ['blueos_nav_core.yaml', 'blueos_aerostack.yaml',
                                  'boat_hardware.yaml', 'submarine_hardware.yaml'])
def test_blueos_ros_nodes_load_camera_sized_dds_transport(name):
    for key, service in services(name).items():
        if service.get('ipc') != 'host':
            continue
        assert 'FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fastdds.xml' in service['environment'], key
        assert '../systems/aerodrone/config/fastdds.xml:/workspace/fastdds.xml:ro' in service['volumes']

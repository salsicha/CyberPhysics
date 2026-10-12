"""Object-tracking wiring (systems/aerodrone/tracking.md): the shared
compositions/aerodrone_tracking.yaml in the aircraft and simulator stacks,
the simulator-only aerodrone_tracking_sim.yaml, their env files, every
profile combination rendering, and the FollowReference behaviour the
tracker flies through."""
import itertools
import json
import os
import re
import shlex
import shutil
import subprocess
from functools import cache, lru_cache

import pytest
import yaml
from conftest import REPO

COMPOSITIONS = REPO / 'compositions'
SIMULATORS = {'aerostack2_sim.yaml': 'gazebo', 'aerostack2_genesis.yaml': 'genesis',
              'aerostack2_isaac.yaml': 'isaac'}
SHM_PROFILE = '/workspace/systems/aerodrone/config/fastdds.xml'
SIM_ONLY = {'tracking_truth_detector', 'tracking_rosbridge', 'tracking_ui'}
TRACKING_PROFILES = ('tracking', 'tracking_truth')


def load(name):
    return yaml.safe_load((COMPOSITIONS / name).read_text())


def env_file(path):
    values = {}
    for line in (REPO / path).read_text().splitlines():
        if line and not line.startswith('#') and '=' in line:
            key, _, value = line.partition('=')
            values[key.strip()] = value.strip()
    return values


def docker_compose_available():
    if not shutil.which('docker'):
        return False
    return subprocess.run(['docker', 'compose', 'version'], capture_output=True).returncode == 0


needs_compose = pytest.mark.skipif(not docker_compose_available(),
                                   reason='Docker Compose is not installed')


def clean_env(**extra):
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('TRACKING_', 'COMPOSE_')) and key not in (
               'DRONE_NAMESPACE', 'VEHICLE_NAMESPACE', 'CYBERPHYSICS_TAG',
               'FASTRTPS_DEFAULT_PROFILES_FILE', 'CAMERA_MOUNT_YAW_DEG',
               'DEMNAV_CACHE_HOST', 'RANGEFINDER_TOPIC')}
    env.update(extra)
    return env


@cache
def rendered(name, profiles=(), **extra):
    args = ['docker', 'compose', '-f', str(COMPOSITIONS / name)]
    for profile in profiles:
        args += ['--profile', profile]
    result = subprocess.run(args + ['config', '--format', 'json'], env=clean_env(**extra),
                            check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def arguments(service):
    """{name: value} of a rendered command's ROS -p name:=value pairs."""
    command = service['command']
    words = command if isinstance(command, list) else shlex.split(command)
    return dict(word.split(':=', 1) for word in words if ':=' in word)


# --- the files ----------------------------------------------------------------
def test_shared_file_holds_only_what_the_aircraft_also_runs():
    shared = load('aerodrone_tracking.yaml')['services']
    assert 'tracker' in shared and not SIM_ONLY & set(shared)
    assert shared['tracker']['profiles'] == list(TRACKING_PROFILES)
    sim = load('aerodrone_tracking_sim.yaml')['services']
    assert set(sim) >= SIM_ONLY
    assert sim['tracking_truth_detector']['profiles'] == ['tracking_truth']
    for name in ('tracking_rosbridge', 'tracking_ui'):
        assert set(sim[name]['profiles']) >= {'tracking'}
    for services in (shared, sim):
        for service in services.values():
            assert service.get('profiles'), 'tracking services are opt-in'


@pytest.mark.parametrize('name', sorted(SIMULATORS))
def test_simulators_include_both_files_with_their_env(name):
    includes = load(name)['include']
    env = [f'../systems/aerodrone/config/{f}' for f in ('topics.env', 'tracking.env')]
    env.append(f'../applications/aerostack2/config/tracking_{SIMULATORS[name]}.env')
    assert {entry['path']: entry['env_file'] for entry in includes} == {
        'aerodrone_tracking.yaml': env, 'aerodrone_tracking_sim.yaml': env}


def test_aircraft_includes_only_the_shared_file_without_env():
    """A nested include's env_file resolves against the working directory,
    so blueos_aerostack.yaml (included by aerodrone_hardware.yaml) passes
    none: the shared file's fallbacks are the aircraft's values."""
    assert load('blueos_aerostack.yaml')['include'] == [
        'blueos_nav_core.yaml', 'aerodrone_tracking.yaml']
    for path in COMPOSITIONS.glob('*.yaml'):
        includes = [entry if isinstance(entry, str) else entry['path']
                    for entry in yaml.safe_load(path.read_text()).get('include') or []]
        if 'aerodrone_tracking_sim.yaml' in includes:
            assert path.name in SIMULATORS, path.name


@pytest.mark.parametrize('name', sorted(SIMULATORS))
def test_simulator_env_matches_the_stack(name):
    """The DEM volume is the stack's DemNav cache, the camera mount and
    stabilisation those DemNav uses."""
    env = env_file(f'applications/aerostack2/config/tracking_{SIMULATORS[name]}.env')
    demnav = load(name)['services']['demnav']
    volume = next(v.split(':')[0] for v in demnav['volumes']
                  if str(v).endswith(':/data/demnav_cache:ro'))
    assert env['TRACKING_DEM_SOURCE'] == volume
    command = demnav['command']
    mount = re.search(r'camera_mount_yaw_deg:=(\S+)', command).group(1)
    assert float(env['TRACKING_CAMERA_MOUNT_YAW_DEG']) == float(mount)
    stabilized = 'camera_stabilized:=true' in command
    assert env['TRACKING_CAMERA_STABILIZED'] == str(stabilized).lower()
    assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == SHM_PROFILE
    assert env['CYBERPHYSICS_TAG'] == 'latest'


def test_every_tracker_setting_in_tracking_env_reaches_the_tracker():
    env = env_file('systems/aerodrone/config/tracking.env')
    command = load('aerodrone_tracking.yaml')['services']['tracker']['command']
    for key, value in env.items():
        assert f'${{{key}:-{value}}}' in command, key


def test_follow_reference_behaviour_runs_with_the_other_behaviours():
    launch = (REPO / 'applications/aerostack2/src/as2_platform_blueos/launch/'
              'blueos_autonomy.launch.py').read_text()
    assert "'follow_reference_behavior_launch.py'" in launch
    assert "'as2_behaviors_motion', 'motion_behaviors_launch.py'" in launch


# --- rendered stacks ------------------------------------------------------------
@needs_compose
@pytest.mark.parametrize('name', sorted(SIMULATORS))
def test_simulator_tracking_services(name):
    services = rendered(name, TRACKING_PROFILES)['services']
    env = env_file(f'applications/aerostack2/config/tracking_{SIMULATORS[name]}.env')
    for service_name in ('tracker', 'tracking_truth_detector'):
        service = services[service_name]
        args = arguments(service)
        assert float(args['camera_mount_yaw_deg']) == float(env['TRACKING_CAMERA_MOUNT_YAW_DEG'])
        assert args['camera_stabilized'] == env['TRACKING_CAMERA_STABILIZED']
        dem = next(v for v in service['volumes'] if v['target'] == '/data/demnav_cache')
        assert dem['type'] == 'volume' and dem['source'] == env['TRACKING_DEM_SOURCE']
        assert dem['read_only']
        assert service['image'] == 'salsicha/cyberphysics:demnav-latest'
    assert arguments(services['tracker'])['namespace'] == 'aerodrone'
    assert arguments(services['tracker'])['topic_prefix'] == '/tracking'
    assert arguments(services['tracking_truth_detector'])['camera_info_topic'] == (
        '/aerodrone/sensor_measurements/downward_rgb/camera_info')
    for service_name in ('tracking_rosbridge', 'tracking_ui'):
        assert services[service_name]['image'] == 'salsicha/cyberphysics:aerostack2-latest'


@needs_compose
@pytest.mark.parametrize('name', sorted(SIMULATORS))
def test_simulator_tracking_ros_containers_load_the_shared_fastdds_profile(name):
    """As every ROS container of the simulator stacks
    (tests/test_aerial_dds_transport.py, which reads only the stack's own
    file)."""
    services = rendered(name, TRACKING_PROFILES)['services']
    tracking = {key: value for key, value in services.items()
                if key == 'tracker' or key.startswith('tracking_')}
    ros = {key: value for key, value in tracking.items()
           if 'ROS_DOMAIN_ID' in (value.get('environment') or {})}
    assert set(ros) >= {'tracker', 'tracking_truth_detector', 'tracking_rosbridge'}
    for key, service in ros.items():
        assert service['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'] == SHM_PROFILE, key
        mount = next(v for v in service['volumes'] if v['target'] == SHM_PROFILE)
        assert mount['source'] == str(REPO / 'systems/aerodrone/config/fastdds.xml')
        assert service.get('ipc') == 'host', key
        assert service.get('network_mode') == 'host', key


@needs_compose
@pytest.mark.parametrize('namespace', [None, 'field_drone'])
def test_aircraft_tracker(namespace):
    extra = {'DRONE_NAMESPACE': namespace} if namespace else {}
    stack = rendered('aerodrone_hardware.yaml', ('tracking',), **extra)['services']
    tracker = stack['tracker']
    args = arguments(tracker)
    assert args['namespace'] == (namespace or 'aerodrone')
    assert args['camera_mount_yaw_deg'] == '0.0'  # topics.env CAMERA_MOUNT_YAW_DEG
    assert args['camera_stabilized'] == 'false'
    assert args['rangefinder_topic'] == '/aerodrone/rangefinder'
    assert tracker['image'] == 'salsicha/cyberphysics:demnav-arm64'
    dem = next(v for v in tracker['volumes'] if v['target'] == '/data/demnav_cache')
    assert dem['type'] == 'bind' and dem['source'] == '/data/demnav_cache'
    assert dem['source'] == next(
        v['source'] for v in stack['demnav']['volumes'] if v['target'] == '/data/demnav_cache')
    # The aircraft keeps FastDDS's defaults, like the rest of its stack.
    assert tracker['environment'].get('FASTRTPS_DEFAULT_PROFILES_FILE') is None


@needs_compose
@pytest.mark.parametrize('profiles', [(), ('tracking',), ('tracking_truth',),
                                      ('tracking', 'tracking_truth')])
def test_aircraft_never_runs_a_second_rosbridge_or_web_page(profiles):
    services = rendered('aerodrone_hardware.yaml', profiles)['services']
    assert not SIM_ONLY & set(services)
    commands = [' '.join(service['command']) if isinstance(service['command'], list)
                else service['command'] for service in services.values()]
    assert sum('rosbridge_websocket_launch' in command for command in commands) == 1
    assert sum('http.server 8080' in command for command in commands) == 1


@needs_compose
def test_blueos_stack_alone_tracks_its_own_namespace():
    tracker = rendered('blueos_aerostack.yaml', ('tracking',))['services']['tracker']
    assert arguments(tracker)['namespace'] == 'drone0'


@needs_compose
@pytest.mark.parametrize('name', sorted(SIMULATORS) + ['aerodrone_hardware.yaml',
                                                        'blueos_aerostack.yaml'])
def test_every_profile_combination_renders(name):
    path = str(COMPOSITIONS / name)
    listed = subprocess.run(['docker', 'compose', '-f', path, 'config', '--profiles'],
                            env=clean_env(), check=True, capture_output=True, text=True)
    profiles = sorted(set(listed.stdout.split()))
    assert set(TRACKING_PROFILES) <= set(profiles)
    for count in range(len(profiles) + 1):
        for combination in itertools.combinations(profiles, count):
            args = ['docker', 'compose', '-f', path]
            for profile in combination:
                args += ['--profile', profile]
            result = subprocess.run(args + ['config', '--quiet'], env=clean_env(),
                                    capture_output=True, text=True)
            assert result.returncode == 0, (combination, result.stderr)

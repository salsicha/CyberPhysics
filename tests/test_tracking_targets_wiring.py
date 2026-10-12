"""Tracking targets in the Gazebo and Isaac compositions: the
models are fetched, baked with the terrain and moved by every simulator,
which publishes /simulation/targets; AERODRONE_MISSION picks the tracking
scenario's tree for flight_ready and the mission alike."""
import re
import shlex
import sys
from types import SimpleNamespace as NS

import pytest
import yaml

from conftest import REPO, load_script

STACKS = {'gazebo': 'aerostack2_sim.yaml', 'isaac': 'aerostack2_isaac.yaml'}
SIMULATOR = {'gazebo': 'gazebo_targets', 'isaac': 'isaacsim'}
TARGETS_JSON = '/workspace/missions/tracking_targets.json'
MODULE_MOUNT = ('../applications/demnav/scripts/tracking_targets.py:'
                '/workspace/applications/demnav/scripts/tracking_targets.py:ro')


def compose(stack):
    return yaml.safe_load((REPO / 'compositions' / STACKS[stack]).read_text())


def options(command):
    words = shlex.split(command)
    return {word[2:]: words[i + 1] for i, word in enumerate(words[:-1])
            if word.startswith('--')}


def interpolate(text, env):
    """Compose's ${VAR:-default} with the given environment."""
    return re.sub(r'\$\{(\w+):-([^}]*)\}', lambda m: env.get(m.group(1), m.group(2)), text)


@pytest.mark.parametrize('stack', sorted(STACKS))
def test_models_are_fetched_into_the_stacks_cache_and_baked_with_the_terrain(stack):
    model = compose(stack)
    services = model['services']
    fetch, terrain = services['target_assets'], services['terrain_assets']
    cache = f'tracking_targets_{stack}_cache'
    assert cache in model['volumes']
    # Fetching needs the network; baking stays offline like the terrain.
    assert fetch['network_mode'] == 'host' and terrain['network_mode'] == 'none'
    assert f'{cache}:/data/target_cache' in fetch['volumes']
    words = shlex.split(fetch['command'])
    assert words[:3] == ['/venv/bin/python3',
                         '/workspace/applications/demnav/scripts/target_models.py', 'fetch']
    assert options(fetch['command']) == {'targets': TARGETS_JSON, 'cache-dir': '/data/target_cache'}
    assert fetch['restart'] == 'no'
    assert terrain['depends_on']['target_assets'] == {
        'condition': 'service_completed_successfully'}
    assert f'{cache}:/data/target_cache:ro' in terrain['volumes']
    assert options(terrain['command'])['targets'] == TARGETS_JSON
    # The builder's default cache directory is where the volume is mounted.
    builder = (REPO / 'applications/demnav/scripts/build_georeferenced_terrain.py').read_text()
    assert 'parser.add_argument("--target-cache", default="/data/target_cache"' in builder
    for service in (fetch, terrain):
        assert '../applications/aerostack2/missions:/workspace/missions:ro' in service['volumes']
        assert ('../applications/demnav/scripts:/workspace/applications/demnav/scripts:ro'
                in service['volumes'])


@pytest.mark.parametrize('stack', sorted(STACKS))
def test_the_simulator_moves_the_targets_with_the_shared_tracks(stack):
    services = compose(stack)['services']
    simulator = services[SIMULATOR[stack]]
    assert MODULE_MOUNT in simulator['volumes']
    assert (REPO / 'applications/demnav/scripts/tracking_targets.py').is_file()
    assets = [v for v in simulator['volumes'] if v.endswith(':/data/navsim_assets:ro')]
    assert assets == [f'navsim_{stack}_assets:/data/navsim_assets:ro']


def test_gazebo_targets_drive_the_world_the_builder_writes():
    services = compose('gazebo')['services']
    driver = services['gazebo_targets']
    assert driver['depends_on'] == ['gazebo']
    assert '../applications/gazebo/scripts:/workspace/scripts:ro' in driver['volumes']
    words = shlex.split(driver['command'])
    assert words[:2] == ['python3', '/workspace/scripts/gazebo_tracking_targets.py']
    builder = (REPO / 'applications/demnav/scripts/build_georeferenced_terrain.py').read_text()
    world = options(driver['command'])['world']
    assert f'<world name="{world}">' in builder and f'world_name: {world}' in builder
    # Its models are on the simulator's resource path.
    assert '/data/navsim_assets/models' in services['gazebo']['environment'][
        'GZ_SIM_RESOURCE_PATH'].split(':')
    # A ROS container of the stack (applications/aerostack2/README.md, "DDS
    # transport").
    assert driver['environment']['ROS_DOMAIN_ID'] == '${ROS_DOMAIN_ID:-0}'
    assert driver['ipc'] == 'host' and driver['network_mode'] == 'host'


@pytest.mark.parametrize('stack', sorted(STACKS))
@pytest.mark.parametrize('mission', [None, 'wilderness', 'tracking'])
def test_aerodrone_mission_picks_one_tree_for_flight_ready_and_the_mission(stack, mission):
    services = compose(stack)['services']
    env = {} if mission is None else {'AERODRONE_MISSION': mission}
    ready = options(interpolate(services['flight_ready']['command'], env))['behavior-tree']
    words = shlex.split(interpolate(services['wilderness_mission']['command'], env))
    tree = dict(word.split(':=', 1) for word in words if ':=' in word)['tree']
    assert ready == tree == f'/data/navsim_assets/{mission or "wilderness"}_mission.xml'


def test_ground_truth_comes_at_10_hz_or_more_of_simulation_time():
    """Isaac publishes it with every camera frame, Gazebo's driver every
    --period-s of simulation time."""
    driver = load_script(REPO / 'applications/gazebo/scripts/gazebo_tracking_targets.py',
                         'gazebo_tracking_targets_review')
    assert driver.parse_args([]).period_s <= 0.1
    isaac = compose('isaac')['services']['isaacsim']['environment']
    assert float(re.search(r':-([\d.]+)', isaac['ISAAC_CAMERA_HZ']).group(1)) >= 10.0


# --- the adapters -----------------------------------------------------------------

def loop_after(source, marker):
    return source[source.index(marker):]


def test_isaac_places_the_targets_before_each_capture_and_publishes_them_after():
    source = (REPO / 'applications/isaac/scripts/aerodrone_navsim.py').read_text()
    main = source[source.index('def main():'):]
    loop = loop_after(main, 'if tick % stride == 0:')
    assert (loop.index('poses = tracks.poses(tick * dt)')
            < loop.index('place_tracking_targets(target_ops, tracks.placed(poses))')
            < loop.index('camera.capture(')
            < loop.index('truth_json(tick * dt, poses, tracks.origin, tracks.hidden)'))
    assert 'publisher.subscribe_hide(tracks)' in main
    add = source[source.index('def add_tracking_targets('):
                 source.index('def place_tracking_targets(')]
    assert 'Physics' not in add and 'Collision' not in add  # no physics on the targets


def test_isaac_publisher_carries_the_truth_topic():
    from unittest import mock

    from conftest import install_ros_stubs
    isaac = load_script(REPO / 'applications/isaac/scripts/aerodrone_navsim.py',
                        'isaac_navsim_tracking_review')
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['rosgraph_msgs.msg'] = mock.MagicMock()
        created = []

        class FakeNode:
            def __init__(self, name):
                pass

            def create_publisher(self, msg_type, topic, qos):
                created.append(topic)
                return mock.MagicMock()

        with mock.patch.object(sys.modules['rclpy.node'], 'Node', FakeNode):
            publisher = isaac.make_publisher(NS(rgb_topic='/rgb', rgb_info_topic='/ri',
                                                depth_topic='/d', depth_info_topic='/di'))
        assert '/simulation/targets' in created
        publisher.publish_targets('{"stamp": 1.0, "targets": []}')
        publisher.targets_publisher.publish.assert_called_once()
        message = publisher.targets_publisher.publish.call_args[0][0]
        assert message.data == '{"stamp": 1.0, "targets": []}'
        subscribed = {}
        publisher.create_subscription = (
            lambda msg_type, topic, callback, qos: subscribed.__setitem__(topic, callback))
        hidden = []
        publisher.subscribe_hide(NS(hide=hidden.append))
        subscribed['/simulation/targets/hide'](NS(data='person_1'))
        assert hidden == ['person_1']


def test_gazebo_driver_updates_every_period_and_after_a_world_reset():
    driver = load_script(REPO / 'applications/gazebo/scripts/gazebo_tracking_targets.py',
                         'gazebo_tracking_targets_schedule')
    schedule = driver.Schedule(0.02)
    due = [t for t in (0.0, 0.005, 0.01, 0.02, 0.025, 0.04, 0.0601, 0.001) if schedule.due(t)]
    assert due == [0.0, 0.02, 0.04, 0.0601, 0.001]
    assert driver.bridge_command('mount_tamalpais') == [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/world/mount_tamalpais/set_pose@ros_gz_interfaces/srv/SetEntityPose']

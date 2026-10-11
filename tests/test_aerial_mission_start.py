"""Aerial mission start: flight_ready waits for the servers the mission tree
calls, and the tree is started again only when it aborted while being built
(as2_behavior_tree gives each action server 1 s then)."""
import functools
import io
import json
import shlex
import signal
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch
from xml.etree import ElementTree

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/aerostack2/scripts'
STACKS = ('aerostack2_sim.yaml', 'aerostack2_genesis.yaml', 'aerostack2_isaac.yaml')
# BT.CPP built-ins the generated tree uses besides the as2 nodes.
BT_BUILTINS = {'Sequence', 'Delay', 'AlwaysSuccess', 'ForceSuccess'}

runner_module = load_script(SCRIPTS / 'run_behavior_tree.py', 'run_behavior_tree_review')


def fake_client(name, kind):
    client = NS(ready=False, name=name, type=kind)
    client.service_is_ready = lambda: client.ready
    client.server_is_ready = lambda: client.ready
    return client


class FakeNode:
    def __init__(self, name):
        self.subscriptions = {}
        self.clients = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_client(self, srv_type, name):
        return self.clients.setdefault(name, fake_client(name, srv_type))

    def get_logger(self):
        return MagicMock()

    def destroy_node(self):
        pass


@pytest.fixture
def ready_module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.action', 'mavros_msgs', 'mavros_msgs.msg', 'mavros_msgs.srv'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        module = load_script(SCRIPTS / 'wait_for_mavros_navigation.py',
                             'wait_for_mavros_navigation_review')
    module.NavSatStatus = NS(STATUS_FIX=0)
    module.ActionClient = lambda node, kind, name: fake_client(name, kind)
    # The interface packages, importable while the test runs.
    interfaces = {'as2_msgs': NS(), 'std_srvs': NS(),
                  'as2_msgs.action': NS(Takeoff='Takeoff', GoToWaypoint='GoToWaypoint',
                                        Land='Land', FollowPath='FollowPath'),
                  'std_srvs.srv': NS(SetBool='SetBool')}
    with patch.dict(sys.modules, interfaces):
        yield module


@functools.cache
def generated_tree():
    """The wilderness mission tree as build_georeferenced_terrain.py writes it."""
    try:
        import cv2  # noqa: F401
    except ModuleNotFoundError:  # only the tree is written, no image is touched
        sys.modules['cv2'] = NS()
    if 'terrain_collisions' not in sys.modules:
        sys.modules['terrain_collisions'] = load_script(
            REPO / 'applications/demnav/scripts/terrain_collisions.py', 'terrain_collisions')
    builder = load_script(REPO / 'applications/demnav/scripts/build_georeferenced_terrain.py',
                          'build_georeferenced_terrain_review')
    mission = json.loads(
        (REPO / 'applications/aerostack2/missions/wilderness_missions.json').read_text())
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        (output / 'mission.json').write_text(json.dumps(mission))
        args = NS(mission=str(output / 'mission.json'), origin_alt=80.0, area_km=8.0,
                  cruise_agl=250.0)
        builder.write_behavior_tree(np.full((9, 9), 100.0, dtype=np.float32), args, output)
        return (output / 'wilderness_mission.xml').read_text()


# --- flight_ready: which servers the tree needs ------------------------------

def test_generated_mission_needs_the_motion_behaviours_and_platform_services(ready_module):
    actions, services = ready_module.behavior_tree_servers(generated_tree(), '/aerodrone')
    assert actions == {
        '/aerodrone/TakeoffBehavior': 'as2_msgs/action/Takeoff',
        '/aerodrone/GoToBehavior': 'as2_msgs/action/GoToWaypoint',
        '/aerodrone/LandBehavior': 'as2_msgs/action/Land'}
    assert services == {
        '/aerodrone/set_arming_state': 'std_srvs/srv/SetBool',
        '/aerodrone/set_offboard_mode': 'std_srvs/srv/SetBool'}


def test_flight_ready_knows_every_node_the_generator_emits(ready_module):
    # A new as2 node in the generated tree must teach flight_ready its server.
    known = set(ready_module.BEHAVIOR_ACTIONS) | set(ready_module.BEHAVIOR_SERVICES)
    tree = ElementTree.fromstring(generated_tree())
    used = {element.get('ID') if element.tag == 'Action' else element.tag
            for element in tree.find('BehaviorTree').iter()} - {'BehaviorTree'}
    assert used - BT_BUILTINS <= known
    assert {'Arm', 'Offboard', 'TakeOff', 'GoTo', 'Land', 'Disarm'} <= used


def test_tree_parsing_follows_remaps_namespaces_and_skips_the_node_model(ready_module):
    xml = """<root main_tree_to_execute="M">
      <BehaviorTree ID="M"><Sequence>
        <TakeOff height="2"/>
        <Action ID="GoTo" server_name="/other/GoToBehavior"/>
        <Action ID="Land" server_name="{land_server}"/>
        <Action ID="Arm" service_name="set_arming_state"/>
        <Action ID="Offboard"/>
        <Delay delay_msec="10"><AlwaysSuccess/></Delay>
      </Sequence></BehaviorTree>
      <TreeNodesModel><Action ID="FollowPath"/></TreeNodesModel>
    </root>"""
    actions, services = ready_module.behavior_tree_servers(xml, 'drone0/')
    assert actions == {'/drone0/TakeoffBehavior': 'as2_msgs/action/Takeoff',
                       '/other/GoToBehavior': 'as2_msgs/action/GoToWaypoint'}
    assert services == {'/drone0/set_arming_state': 'std_srvs/srv/SetBool'}
    assert ready_module.resolve_name('TakeoffBehavior', '/') == '/TakeoffBehavior'
    assert ready_module.resolve_name('/abs', '/aerodrone') == '/abs'


# --- flight_ready: readiness ---------------------------------------------------

def readiness(module, actions=None, services=None):
    return module.NavigationReadiness('/mavros', actions, services)


def navigation_up(node):
    node.state = NS(connected=True)
    node.intervals_configured = True
    node.fix = NS(status=NS(status=0), latitude=37.9, longitude=-122.6)
    node.odom = NS(pose=NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.1))))


def test_ready_needs_mavros_navigation_and_every_mission_server(ready_module):
    actions, services = ready_module.behavior_tree_servers(generated_tree(), '/aerodrone')
    node = readiness(ready_module, actions, services)
    assert node.waiting_for()[:3] == ['MAVROS connection', 'GPS fix', 'local odometry']
    navigation_up(node)
    assert not node.ready()
    assert node.waiting_for() == sorted(actions) + sorted(services)
    assert {name: client.type for name, client in node.action_clients.items()} == {
        '/aerodrone/TakeoffBehavior': 'Takeoff', '/aerodrone/GoToBehavior': 'GoToWaypoint',
        '/aerodrone/LandBehavior': 'Land'}
    assert {client.type for client in node.service_clients.values()} == {'SetBool'}
    for client in node.action_clients.values():
        client.ready = True
    assert node.waiting_for() == sorted(services)
    for client in node.service_clients.values():
        client.ready = True
    assert node.ready()
    # The platform is up but a behaviour is gone again: not ready.
    node.action_clients['/aerodrone/TakeoffBehavior'].ready = False
    assert node.waiting_for() == ['/aerodrone/TakeoffBehavior']


def test_navigation_only_readiness_is_unchanged_without_a_tree(ready_module):
    node = readiness(ready_module)
    navigation_up(node)
    assert node.ready()
    node.intervals_configured = False
    assert node.waiting_for() == ['MAVLink telemetry intervals']
    node.intervals_configured = True
    node.fix.status.status = -1
    assert node.waiting_for() == ['GPS fix']
    node.fix.status.status = 0
    node.odom.pose.pose.position.z = float('nan')
    assert node.waiting_for() == ['local odometry']


def test_gps_fix_is_optional_for_a_vehicle_that_may_start_gps_denied(ready_module):
    assert ready_module.parse_args([]).require_gps
    assert not ready_module.parse_args(['--no-require-gps']).require_gps
    node = ready_module.NavigationReadiness('/mavros', require_gps=False)
    navigation_up(node)
    node.fix = None
    assert node.ready()
    node.odom.pose.pose.position.x = float('nan')
    assert node.waiting_for() == ['local odometry']


def run_main(module, tree_xml, servers_up_after_s, timeout_s=60.0, extra_args=()):
    """main() against a fake clock; the servers appear after a delay."""
    clock = NS(t=0.0)
    nodes = []

    def spin_once(node, timeout_sec):
        clock.t += timeout_sec
        navigation_up(node)
        node.configure_intervals = lambda: True
        for client in list(node.action_clients.values()) + list(node.service_clients.values()):
            client.ready = clock.t >= servers_up_after_s

    module.rclpy = NS(init=lambda: None, ok=lambda: True, spin_once=spin_once,
                      shutdown=lambda: None)
    module.time = NS(monotonic=lambda: clock.t)
    real = module.NavigationReadiness

    def make(*args, **kwargs):
        node = real(*args, **kwargs)
        nodes.append(node)
        return node

    with tempfile.NamedTemporaryFile('w', suffix='.xml') as tree, \
            patch.object(module, 'NavigationReadiness', make):
        tree.write(tree_xml)
        tree.flush()
        module.main(['--timeout', str(timeout_s), '--settle-seconds', '3',
                     '--behavior-tree', tree.name, '--behavior-namespace', '/aerodrone']
                    + list(extra_args))
    return clock.t, nodes[0]


def test_flight_ready_completes_only_once_the_servers_are_up_and_settled(ready_module):
    elapsed, node = run_main(ready_module, generated_tree(), servers_up_after_s=20.0)
    assert 23.0 <= elapsed < 24.0
    assert len(node.action_clients) == 3 and len(node.service_clients) == 2


def test_zero_timeout_waits_as_long_as_it_takes(ready_module):
    elapsed, _ = run_main(ready_module, generated_tree(), servers_up_after_s=3600.0,
                          timeout_s=0.0, extra_args=['--no-require-gps'])
    assert 3603.0 <= elapsed < 3604.0


def test_flight_ready_times_out_naming_the_missing_servers(ready_module):
    with pytest.raises(RuntimeError, match='/aerodrone/TakeoffBehavior'):
        run_main(ready_module, generated_tree(), servers_up_after_s=1e9, timeout_s=10.0)


# --- run_behavior_tree.py ----------------------------------------------------------

# Lines of the Isaac run whose tree aborted while being built.
CONSTRUCTION_FAILURE = [
    '[INFO] [as2_behavior_tree_node-1]: process started with pid [22]\n',
    ('[as2_behavior_tree_node-1] \x1b[31m[ERROR] [1791631869.540066374] [aerodrone.bt_manager]: '
     '"TakeoffBehavior" action server not available after waiting for 1 s\x1b[0m\n'),
    ("[as2_behavior_tree_node-1] terminate called after throwing an instance of "
     "'std::length_error'\n"),
    '[as2_behavior_tree_node-1]   what():  basic_string::_M_create\n',
    ("[ERROR] [as2_behavior_tree_node-1]: process has died [pid 22, exit code -6, cmd "
     "'/opt/ros/humble/lib/as2_behavior_tree/as2_behavior_tree_node']\n"),
]
TICK = ('[as2_behavior_tree_node-1] [1791632566.377]: Sequence                  '
        '\x1b[36mIDLE\x1b[0m -> \x1b[33mRUNNING\x1b[0m\n')
MISSION_DONE = [
    TICK,
    ('[as2_behavior_tree_node-1] [1791648703.241]: Sequence                  '
     '\x1b[33mRUNNING\x1b[0m -> \x1b[32mSUCCESS\x1b[0m\n'),
    '[INFO] [as2_behavior_tree_node-1]: process has finished cleanly [pid 22]\n',
]
CRASH_IN_FLIGHT = [
    TICK,
    "[ERROR] [as2_behavior_tree_node-1]: process has died [pid 22, exit code -11, cmd 'x']\n",
]


class FakeChild:
    def __init__(self, lines, returncode=0, on_line=None):
        self.lines = lines
        self.returncode = returncode
        self.on_line = on_line
        self.signals = []
        self.done = False

    @property
    def stdout(self):
        for number, line in enumerate(self.lines):
            if self.on_line:
                self.on_line(number)
            yield line
        self.done = True

    def poll(self):
        return self.returncode if self.done else None

    def send_signal(self, signum):
        self.signals.append(signum)

    def wait(self):
        self.done = True
        return self.returncode


def tree_runner(children, attempts=5, retry_delay=5.0):
    launched = []

    def popen(command, **kwargs):
        assert kwargs['start_new_session'] and kwargs['stderr'] == runner_module.subprocess.STDOUT
        child = children[len(launched)]
        launched.append(command)
        return child

    clock = NS(t=0.0)

    def sleep(seconds):
        clock.t += seconds

    out = io.StringIO()
    runner = runner_module.TreeRunner(['ros2', 'launch', 'x'], attempts, retry_delay,
                                      popen=popen, clock=lambda: clock.t, sleep=sleep, out=out)
    return runner, launched, out, clock


def test_attempt_recognises_the_construction_abort_and_a_ticked_tree():
    attempt = runner_module.Attempt()
    for line in CONSTRUCTION_FAILURE:
        attempt.observe(line)
    assert attempt.never_started and attempt.process_died
    attempt.observe(TICK)
    assert not attempt.never_started


def test_tree_that_never_started_is_started_again_until_it_runs():
    runner, launched, out, clock = tree_runner(
        [FakeChild(CONSTRUCTION_FAILURE), FakeChild(CONSTRUCTION_FAILURE), FakeChild(MISSION_DONE)])
    assert runner.run() == 0
    assert len(launched) == 3
    assert clock.t == pytest.approx(10.0)
    assert out.getvalue().count('before its first tick (attempt') == 2
    assert 'basic_string::_M_create' in out.getvalue()  # the output is passed through


def test_tree_that_never_builds_fails_after_its_attempts():
    runner, launched, out, _ = tree_runner([FakeChild(CONSTRUCTION_FAILURE)] * 3, attempts=3)
    assert runner.run() == 1
    assert len(launched) == 3
    assert 'could not be built in 3 attempts' in out.getvalue()


def test_tree_that_ticked_is_never_started_again():
    # Even a construction-time message after the first tick is no retry.
    runner, launched, _, _ = tree_runner([FakeChild(CRASH_IN_FLIGHT + CONSTRUCTION_FAILURE)])
    assert runner.run() == 1  # its process died: non-zero
    assert len(launched) == 1
    runner, launched, _, _ = tree_runner([FakeChild(MISSION_DONE)])
    assert runner.run() == 0 and len(launched) == 1


def test_stop_signal_is_passed_on_and_ends_the_run():
    holder = {}
    child = FakeChild(CONSTRUCTION_FAILURE, on_line=lambda number: number == 1 and
                      holder['runner'].stop(signal.SIGINT))
    runner, launched, _, _ = tree_runner([child, FakeChild(MISSION_DONE)])
    holder['runner'] = runner
    assert runner.run() == 0
    assert child.signals == [signal.SIGINT]
    assert len(launched) == 1  # no retry after a stop


def test_stop_while_waiting_to_retry_starts_nothing_new():
    runner, launched, _, clock = tree_runner(
        [FakeChild(CONSTRUCTION_FAILURE), FakeChild(MISSION_DONE)])
    real_sleep = runner.sleep

    def sleep(seconds):
        real_sleep(seconds)
        if clock.t >= 1.0:
            runner.stop(signal.SIGTERM)
    runner.sleep = sleep
    runner.run()
    assert len(launched) == 1 and clock.t < 5.0


def test_exit_status_of_a_signalled_command():
    assert runner_module.exit_status(-6) == 134
    assert runner_module.exit_status(0) == 0


# --- compose wiring -----------------------------------------------------------------

def services(name):
    return yaml.safe_load((REPO / 'compositions' / name).read_text())['services']


def launch_args(tokens):
    return dict(token.split(':=', 1) for token in tokens if ':=' in token)


@pytest.mark.parametrize('name', STACKS)
def test_mission_waits_for_flight_ready_which_waits_for_the_tree_servers(name, ready_module):
    stack = services(name)
    mission, ready = stack['wilderness_mission'], stack['flight_ready']
    assert mission['depends_on']['flight_ready'] == {
        'condition': 'service_completed_successfully'}

    # The tree the mission runs, in the namespace it runs in.
    wrapper = runner_module.parse_args(shlex.split(mission['command'])[2:])
    assert shlex.split(mission['command'])[1].endswith('/run_behavior_tree.py')
    assert wrapper.attempts >= 3 and wrapper.retry_delay > 0
    assert wrapper.command[:4] == ['ros2', 'launch', 'as2_behavior_tree',
                                   'behavior_trees.launch.py']
    tree_args = launch_args(wrapper.command)

    # flight_ready reads that same tree from the same volume.
    args = ready_module.parse_args(shlex.split(ready['command'])[2:])
    assert shlex.split(ready['command'])[1].endswith('/wait_for_mavros_navigation.py')
    assert args.behavior_tree == tree_args['tree']
    assert args.behavior_namespace.strip('/') == tree_args['drone_id']
    assets = [volume for volume in mission['volumes'] if volume.endswith(':/data/navsim_assets:ro')]
    assert len(assets) == 1 and assets[0] in ready['volumes']
    assert ready['depends_on']['terrain_assets'] == {
        'condition': 'service_completed_successfully'}
    assert 'aerostack2_autonomy' in ready['depends_on']

    # Both scripts come from the repository, not the image.
    scripts = '../applications/aerostack2/scripts:/workspace/applications/aerostack2/scripts:ro'
    assert scripts in mission['volumes'] and scripts in ready['volumes']
    # docker stop reaches the wrapper, which passes SIGINT on to ros2 launch.
    assert mission['stop_signal'] == 'SIGINT'


def test_drone_tree_waits_for_flight_ready_without_gps_or_timeout(ready_module):
    """The BlueOS drone (blueos_aerostack.yaml) gates its map-goal tree the
    same way, but may start GPS-denied and must not give up on a slow start."""
    stack = services('blueos_aerostack.yaml')
    tree, ready = stack['behavior_tree'], stack['flight_ready']
    assert tree['depends_on']['flight_ready'] == {'condition': 'service_completed_successfully'}
    assert tree['restart'] == 'always'  # the tree ends after each map goal
    assert tree['stop_signal'] == 'SIGINT'

    wrapper = runner_module.parse_args(shlex.split(tree['command'])[2:])
    assert shlex.split(tree['command'])[1].endswith('/run_behavior_tree.py')
    assert wrapper.command[:4] == ['ros2', 'launch', 'as2_behavior_tree',
                                   'behavior_trees.launch.py']
    tree_args = launch_args(wrapper.command)

    args = ready_module.parse_args(shlex.split(ready['command'])[2:])
    assert args.behavior_tree == tree_args['tree']
    assert args.behavior_namespace.strip('/') == tree_args['drone_id']
    assert not args.require_gps
    assert args.timeout <= 0
    assert set(ready['depends_on']) >= {'mavros', 'aerostack'}
    scripts = '../applications/aerostack2/scripts:/workspace/applications/aerostack2/scripts:ro'
    assert scripts in tree['volumes'] and scripts in ready['volumes']

    # The tree ships in the image: the servers it needs are the GoTo behaviour.
    source = REPO / 'applications/aerostack2/src/as2_platform_blueos/resource/map_goal.xml'
    assert tree_args['tree'].endswith('/as2_platform_blueos/resource/map_goal.xml')
    actions, services_needed = ready_module.behavior_tree_servers(
        source.read_text(), args.behavior_namespace)
    assert list(actions) == ['/${DRONE_NAMESPACE:-drone0}/GoToBehavior']
    assert services_needed == {}

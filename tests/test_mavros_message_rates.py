"""The BlueOS Aerodrone, airplane, boat and submarine stacks keep asking the
autopilot for the MAVLink message rates their bridges and navigation need.

On a Raspberry Pi 5 Navigator under BlueOS, MAVROS's /mavros/imu/data and
every other stream of these stacks ran at about 1 Hz: a fresh ArduPlane or
ArduRover streams at its SR0_* rates (all 1 Hz on ArduPlane 4.6.3), and
nothing asked for more; ArduSub's defaults gave attitude 10 Hz but RAW_IMU,
the pressure and GPS_RAW_INT 2 Hz. Each stack now runs
mavros_message_rates.py, which sends MAV_CMD_SET_MESSAGE_INTERVAL through
MAVROS (the request the Aerodrone's flight_ready makes, through the same
function) for the messages in systems/<platform>/config/message_rates.yaml,
at start and again after a reconnect or an autopilot restart. Measured on
the Pi: imu/data 1 -> 20 Hz (ArduPlane), 1 -> 10 Hz (ArduRover), and after
an autopilot restart the rates came back within seconds.

The Aerodrone's flight_ready asks for its rates once and exits, so an
autopilot reboot mid-session left the drone at ArduCopter's own stream
rates; its BlueOS stack now runs the same keeper with flight_ready's rates.
"""
import itertools
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/aerostack2/scripts'
KEEPER = SCRIPTS / 'mavros_message_rates.py'
# platform: (composition, --env-file of its README run, rates service, MAVROS
# service, the MAVROS client beside it (its bridge), bridge parameter file)
STACKS = {
    'aerodrone': ('aerodrone_hardware.yaml', None, 'aerodrone_message_rates', 'mavros',
                  'aerostack', None),
    'airplane': ('airplane_hardware.yaml', 'systems/airplane/config/blueos.env',
                 'airplane_message_rates', 'mavros', 'airplane_bridge',
                 'systems/airplane/config/mavros_bridge.yaml'),
    'boat': ('boat_hardware.yaml', 'systems/boat/config/blueos.env',
             'boat_message_rates', 'boat_mavros', 'boat_bridge',
             'systems/boat/config/bridge.yaml'),
    'submarine': ('submarine_hardware.yaml', 'systems/submarine/config/blueos.env',
                  'submarine_message_rates', 'submarine_mavros', 'submarine_bridge',
                  'systems/submarine/config/bridge.yaml'),
}
# MAVROS 2.14 topic -> the MAVLink messages it is made from (every one of
# them must stream at the topic's rate).
MAVROS_SOURCES = {
    'imu/data': ('ATTITUDE', 'ATTITUDE_QUATERNION', 'RAW_IMU'),
    'local_position/odom': ('LOCAL_POSITION_NED', 'ATTITUDE'),
    'global_position/global': ('GLOBAL_POSITION_INT', 'GPS_RAW_INT'),
    'battery': ('BATTERY_STATUS',),
    'imu/static_pressure': ('SCALED_PRESSURE',),
}
# What the vehicles sent before (measured on the Pi 5 Navigator, Hz).
ARDUPILOT_DEFAULT_HZ = 1.0
NAVIGATION_MIN_HZ = 10.0


# --- loading the scripts without ROS -----------------------------------------

class FakeLogger:
    def __init__(self):
        self.lines = []

    def _log(self, level, message, **_):
        self.lines.append((level, message))

    def info(self, message, **kwargs):
        self._log('info', message)

    def warning(self, message, **kwargs):
        self._log('warning', message)

    warn = warning


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.subscriptions = {}
        self.clients = {}
        self.publishers = []
        self.logger = FakeLogger()

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_client(self, srv_type, name):
        client = NS(name=name, type=srv_type, ready=True)
        client.service_is_ready = lambda: client.ready
        self.clients[name] = client
        return client

    def create_publisher(self, *args, **kwargs):
        self.publishers.append(args)

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


@pytest.fixture
def modules():
    """(wait_for_mavros_navigation, mavros_message_rates) on ROS stubs."""
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.action', 'mavros_msgs', 'mavros_msgs.msg', 'mavros_msgs.srv'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        ready = load_script(SCRIPTS / 'wait_for_mavros_navigation.py',
                            'wait_for_mavros_navigation')
        sys.modules['wait_for_mavros_navigation'] = ready
        keeper = load_script(KEEPER, 'mavros_message_rates_review')
    return ready, keeper


@pytest.fixture
def keeper_module(modules):
    return modules[1]


class Autopilot:
    """request_message_interval stand-in: records each request and answers
    from `answers` ({message id: [results...]}, default accepted)."""

    def __init__(self, answers=None, on_request=None):
        self.requests = []
        self.answers = {key: list(value) for key, value in (answers or {}).items()}
        self.on_request = on_request

    def __call__(self, node, client, message_id, rate_hz, timeout_sec=5.0):
        assert client.name == '/mavros/set_message_interval'
        self.requests.append((message_id, rate_hz))
        if self.on_request:
            self.on_request(node, message_id)
        queue = self.answers.get(message_id)
        return queue.pop(0) if queue else True


def keeper_with(module, rates, autopilot, **kwargs):
    module.request_message_interval = autopilot
    keeper = module.MessageRateKeeper('/mavros', rates, **kwargs)
    keeper.autopilot = autopilot
    return keeper


def connect(keeper, connected=True):
    keeper.subscriptions['/mavros/state'](NS(connected=connected))


def stream_imu(keeper, now, seconds, rate_hz, rounds=None):
    """imu/data at rate_hz for `seconds`, stepping the keeper every 0.1 s as
    main() does; appends the time of each request round to `rounds`.
    Returns the time reached."""
    autopilot = keeper.autopilot
    seen = len(autopilot.requests)
    owed = 0.0
    for _ in range(round(seconds * 10)):
        now = round(now + 0.1, 6)
        owed += rate_hz / 10.0
        while owed >= 1.0 - 1e-9:
            keeper.subscriptions['/mavros/imu/data'](NS())
            owed -= 1.0
        keeper.step(now)
        if len(autopilot.requests) > seen and rounds is not None:
            rounds.append(now)
        seen = len(autopilot.requests)
    return now


def platform_rates(module, platform):
    return module.load_rates(REPO / f'systems/{platform}/config/message_rates.yaml')


def by_name(module, rates):
    return {module.message_name(identifier): rate for identifier, rate in rates.items()}


# --- the stacks run it --------------------------------------------------------

def rendered_services(composition, env_file):
    if not shutil.which('docker') or subprocess.run(
            ['docker', 'compose', 'version'], capture_output=True).returncode:
        pytest.skip('Docker Compose is not installed')
    env = {key: value for key, value in os.environ.items()
           if key not in ('COMPOSE_FILE', 'COMPOSE_ENV_FILES', 'BLUEOS_FCU_URL')}
    args = ['docker', 'compose']
    if env_file:
        args += ['--env-file', str(REPO / env_file)]
    rendered = subprocess.run(
        args + ['-f', str(REPO / 'compositions' / composition), 'config', '--format', 'json'],
        env=env, check=True, capture_output=True, text=True)
    return json.loads(rendered.stdout)['services']


def host_path(service, container_path):
    """The host file a bind mount puts at container_path."""
    for volume in service['volumes']:
        target = volume['target'].rstrip('/')
        if container_path == target or container_path.startswith(target + '/'):
            return Path(volume['source']) / os.path.relpath(container_path, target)
    raise AssertionError(f'{container_path} is not mounted')


@pytest.mark.parametrize('platform', sorted(STACKS))
def test_each_hardware_stack_keeps_its_platform_rates(platform):
    composition, env_file, name, mavros, bridge, _ = STACKS[platform]
    services = rendered_services(composition, env_file)
    rates, bridge_service = services[name], services[bridge]
    args = rates['command']
    assert args[:2] == [
        'python3', '/workspace/applications/aerostack2/scripts/mavros_message_rates.py']
    assert host_path(rates, args[1]) == KEEPER
    # the script imports the flight_ready request function beside it
    assert host_path(rates, '/workspace/applications/aerostack2/scripts/'
                            'wait_for_mavros_navigation.py').is_file()
    assert host_path(rates, args[args.index('--rates') + 1]) == (
        REPO / f'systems/{platform}/config/message_rates.yaml')
    assert args[args.index('--namespace') + 1] == '/mavros'
    assert 'fcu_url:=tcp://127.0.0.1:5777' in services[mavros]['command']
    assert mavros in rates['depends_on']
    # beside the bridge: the stack's own image, host network and IPC, and
    # back after a crash or a Pi reboot
    assert rates['image'] == bridge_service['image']
    assert rates['network_mode'] == 'host' and rates['ipc'] == 'host'
    assert rates['restart'] == 'unless-stopped'
    assert rates['container_name'] == name


@pytest.mark.parametrize('platform', sorted(name for name in STACKS if STACKS[name][5]))
def test_each_rates_file_covers_every_topic_its_bridge_mirrors(platform, keeper_module):
    """Every MAVROS topic the platform's mavros_mirror republishes is made
    from messages requested well above ArduPilot's 1 Hz default, the
    navigation inputs (attitude, IMU, odometry) at 10 Hz or more."""
    bridge_file = STACKS[platform][5]
    parameters = yaml.safe_load((REPO / bridge_file).read_text())['/**']['ros__parameters']
    source = (REPO / 'applications/aerostack2/mavros_mirror/mavros_mirror/__init__.py').read_text()
    mirrored = re.findall(r'f"\{mavros_namespace\}/([a-z_/]+)"', source)
    assert sorted(mirrored) == sorted(['global_position/global', 'local_position/odom',
                                       'imu/data', 'battery'])
    if parameters.get('mirror_pressure'):
        prefix = parameters['mavros_namespace'].rstrip('/') + '/'
        mirrored.append(parameters['pressure_topic'].removeprefix(prefix))
    rates = by_name(keeper_module, platform_rates(keeper_module, platform))
    for topic in mirrored:
        for message in MAVROS_SOURCES[topic]:
            assert rates.get(message, 0.0) > ARDUPILOT_DEFAULT_HZ, (platform, topic, message)
    for message in ('ATTITUDE', 'ATTITUDE_QUATERNION', 'RAW_IMU', 'LOCAL_POSITION_NED'):
        assert rates[message] >= NAVIGATION_MIN_HZ, (platform, message)
    # MAVROS publishes imu/data from whichever attitude message it has seen,
    # with the latest RAW_IMU's acceleration, and stamps odometry with the
    # latest attitude: one rate for all four.
    assert len({rates[message] for message in (
        'ATTITUDE', 'ATTITUDE_QUATERNION', 'RAW_IMU', 'LOCAL_POSITION_NED')}) == 1
    assert rates['VFR_HUD'] > ARDUPILOT_DEFAULT_HZ
    assert ('SCALED_PRESSURE' in rates) == bool(parameters.get('mirror_pressure'))
    # The submarine's pressure is its external depth sensor's message
    # (SCALED_PRESSURE2, decoded by the bridge), barometer 0 the fallback:
    # both at one rate (tests/test_submarine_depth_pressure.py).
    if parameters.get('pressure_source') == 'ardusub_depth_sensor':
        depth = parameters['depth_sensor_message']
        assert rates[depth] == rates['SCALED_PRESSURE'] >= NAVIGATION_MIN_HZ
    else:
        assert 'SCALED_PRESSURE2' not in rates


def test_the_airplane_gets_the_rates_its_simulation_runs_at(modules):
    """airplane_sim.yaml's airplane_navigation_ready asks ArduPlane SITL for
    flight_ready's rates; the aircraft gets the same attitude and odometry
    rates, which DemNav's and WildNav's attitude history were tuned on."""
    ready, keeper = modules
    sim = yaml.safe_load((REPO / 'compositions/airplane_sim.yaml').read_text())
    command = shlex.split(sim['services']['airplane_navigation_ready']['command'])
    assert command[1].endswith('/wait_for_mavros_navigation.py')
    airplane = platform_rates(keeper, 'airplane')
    for message in ('ATTITUDE', 'ATTITUDE_QUATERNION', 'LOCAL_POSITION_NED'):
        identifier = keeper.MESSAGE_IDS[message]
        assert airplane[identifier] == ready.MESSAGE_RATES_HZ[identifier] == 20.0


def test_the_aerodrone_keeps_the_rates_flight_ready_asks_for(modules):
    """flight_ready (wait_for_mavros_navigation.py) still asks once, before
    the map-goal tree starts, and exits; aerodrone_message_rates keeps the
    same rates for the rest of the session, through autopilot reboots."""
    ready, keeper = modules
    rates = platform_rates(keeper, 'aerodrone')
    assert rates == ready.MESSAGE_RATES_HZ
    stack = yaml.safe_load((REPO / 'compositions/blueos_aerostack.yaml').read_text())
    flight_ready = stack['services']['flight_ready']
    assert 'wait_for_mavros_navigation.py' in flight_ready['command']
    assert flight_ready['restart'] == 'no'
    # The keeper watches imu/data at the 20 Hz both ask for.
    assert keeper_with(keeper, rates, Autopilot()).attitude_rate_hz() == 20.0


# BlueOS composition: (--env-file of its README run, the rate keepers it runs)
BLUEOS_KEEPERS = {
    'aerodrone_hardware.yaml': (None, {'aerodrone_message_rates'}),
    'blueos_aerostack.yaml': (None, {'aerodrone_message_rates'}),
    'blueos_drone.yaml': (None, {'aerodrone_message_rates'}),
    'airplane_hardware.yaml': (STACKS['airplane'][1], {'airplane_message_rates'}),
    'boat_hardware.yaml': (STACKS['boat'][1], {'boat_message_rates'}),
    'submarine_hardware.yaml': (STACKS['submarine'][1], {'submarine_message_rates'}),
    # Included by the drone and the airplane, each with its own keeper.
    'blueos_nav_core.yaml': (None, set()),
}


@pytest.mark.parametrize('composition', sorted(BLUEOS_KEEPERS))
def test_every_blueos_mavros_has_one_rate_keeper(composition):
    """One keeper per MAVROS: the shared navigation core runs MAVROS for the
    drone (blueos_aerostack.yaml) and the airplane, and each adds its own
    rates; a keeper in the core would ask the airplane for the drone's."""
    env_file, expected = BLUEOS_KEEPERS[composition]
    services = rendered_services(composition, env_file)
    keepers = {name for name, service in services.items()
               if KEEPER.name in ' '.join(service.get('command') or [])}
    assert keepers == expected
    assert sum('apm.launch' in ' '.join(service.get('command') or [])
               for service in services.values()) == 1


@pytest.mark.parametrize('composition', ['aerostack2_sim.yaml', 'aerostack2_isaac.yaml'])
def test_the_aerodrone_simulations_ask_once(composition):
    """ArduCopter SITL starts with these stacks and nothing reboots it under
    them, so flight_ready's one request holds. A keeper there would also
    measure imu/data in wall-clock time and take a simulation running
    slower than half real time for lost rates."""
    services = yaml.safe_load((REPO / 'compositions' / composition).read_text())['services']
    commands = {name: str(service.get('command', '')) for name, service in services.items()}
    assert not any(KEEPER.name in command for command in commands.values())
    assert 'wait_for_mavros_navigation.py' in commands['flight_ready']


@pytest.mark.parametrize('platform', sorted(STACKS))
def test_each_platform_readme_documents_its_rates(platform):
    text = (REPO / f'systems/{platform}/README.md').read_text()
    assert '## Telemetry rates' in text
    section = text.split('## Telemetry rates', 1)[1].split('\n## ', 1)[0]
    assert 'config/message_rates.yaml' in section
    assert STACKS[platform][2] in section


# --- rates files ---------------------------------------------------------------

@pytest.mark.parametrize('platform', sorted(STACKS))
def test_rates_files_name_known_messages_at_sane_rates(platform, keeper_module):
    rates = platform_rates(keeper_module, platform)
    assert all(identifier in keeper_module.MESSAGE_NAMES for identifier in rates)
    assert all(0.0 < rate <= keeper_module.MAX_RATE_HZ for rate in rates.values())


def test_rates_accept_names_or_ids(keeper_module):
    parse = keeper_module.parse_rates
    assert parse({'message_rates_hz': {'attitude': 20, 'GPS_RAW_INT': '5', 147: 2,
                                       '245': 0.5}}) == {30: 20.0, 24: 5.0, 147: 2.0,
                                                         245: 0.5}


@pytest.mark.parametrize('entries,error', [
    ({'ATTITUDE': 0}, 'rates must be'),
    ({'ATTITUDE': -1}, 'rates must be'),
    ({'ATTITUDE': math.nan}, 'rates must be'),
    ({'ATTITUDE': 200}, 'rates must be'),       # an interval in ms, not Hz
    ({'ATTITUDE': 20, 30: 10}, 'listed twice'),
    ({'ATITUDE': 20}, 'unknown MAVLink message'),
    ({}, 'no message_rates_hz'),
    (None, 'no message_rates_hz'),
])
def test_rates_files_with_mistakes_are_refused(keeper_module, entries, error):
    with pytest.raises(ValueError, match=error):
        keeper_module.parse_rates({'message_rates_hz': entries})


# --- when it asks ----------------------------------------------------------------

RATES = {30: 20.0, 31: 20.0, 27: 20.0, 32: 20.0, 33: 5.0, 147: 2.0}


def test_nothing_is_asked_before_mavros_connects_then_every_rate_once(keeper_module):
    autopilot = Autopilot()
    keeper = keeper_with(keeper_module, RATES, autopilot)
    stream_imu(keeper, 0.0, 10.0, 0.0)
    assert autopilot.requests == []
    connect(keeper)
    now = stream_imu(keeper, 0.0, 0.1, 0.0)
    assert autopilot.requests == list(RATES.items())
    stream_imu(keeper, now, 60.0, 20.0)
    assert autopilot.requests == list(RATES.items())
    assert ('info', 'imu/data at 20.0 Hz (requested 20 Hz)') in keeper.logger.lines


def test_a_reconnect_asks_again(keeper_module):
    """MAVROS reconnects after its heartbeat timeout: an autopilot reboot
    longer than that, or the link (BlueOS restarts its router with the
    autopilot). Measured on the Pi: 16 s without heartbeat, then
    reconnected and every rate requested again."""
    autopilot = Autopilot()
    keeper = keeper_with(keeper_module, RATES, autopilot)
    connect(keeper)
    now = stream_imu(keeper, 0.0, 10.0, 20.0)
    connect(keeper, False)
    assert keeper.logger.lines[-1][0] == 'warning'
    now = stream_imu(keeper, now, 15.0, 0.0)
    assert len(autopilot.requests) == len(RATES)
    connect(keeper)
    stream_imu(keeper, now, 0.1, 0.0)
    assert autopilot.requests == 2 * list(RATES.items())


def test_a_dropped_attitude_stream_asks_again_and_backs_off(keeper_module):
    """An autopilot restart shorter than MAVROS's heartbeat timeout, or a
    ground station resetting the streams, leaves MAVROS connected with
    imu/data at the autopilot's default rate. On the Pi 5 an ArduSub whose
    attitude was set back to 1 Hz was at 10 Hz again 10 s later."""
    autopilot = Autopilot()
    keeper = keeper_with(keeper_module, RATES, autopilot)
    connect(keeper)
    rounds = []
    now = stream_imu(keeper, 0.0, 30.0, 20.0, rounds)
    assert rounds == [0.1]
    # The rates are gone: caught within two 5 s windows (the one the drop
    # falls in may still average enough) ...
    drop, rounds = now, []
    now = stream_imu(keeper, now, 200.0, 1.0, rounds)
    assert rounds[0] - drop <= 10.3
    assert any('imu/data at 0.' in line or 'imu/data at 1.' in line
               for level, line in keeper.logger.lines if level == 'warning')
    # ... then, while they stay low, asked for after 10, 20, 40 and then
    # every 60 s instead of every 5 s.
    gaps = [later - earlier for earlier, later in itertools.pairwise(rounds)]
    assert gaps == pytest.approx([10.2, 20.2, 40.2, 60.2, 60.2], abs=0.15)
    # Back at the requested rate, the checks return to every 5 s.
    now = stream_imu(keeper, now, 130.0, 20.0)
    assert keeper.backoff == 1.0
    drop, rounds = now, []
    stream_imu(keeper, now, 11.0, 0.0, rounds)
    assert len(rounds) == 1 and rounds[0] - drop <= 10.3


def test_unaccepted_requests_are_retried_then_given_up(keeper_module):
    """Right after an autopilot restart the first request went unanswered
    (Pi 5 Navigator); a message the autopilot never accepts is dropped
    after three attempts instead of being asked for every 2 s."""
    autopilot = Autopilot({30: [False], 31: [False, False, False]})
    keeper = keeper_with(keeper_module, RATES, autopilot, retry_seconds=2.0)
    connect(keeper)
    keeper.step(0.0)
    assert autopilot.requests == list(RATES.items())
    keeper.step(1.9)
    assert len(autopilot.requests) == len(RATES)
    keeper.step(2.0)
    assert autopilot.requests[len(RATES):] == [(30, 20.0), (31, 20.0)]
    keeper.step(4.0)
    assert autopilot.requests[len(RATES) + 2:] == [(31, 20.0)]
    assert keeper.pending == {} and keeper.refused == {31}
    assert any('did not accept ATTITUDE_QUATERNION' in line
               for level, line in keeper.logger.lines if level == 'warning')
    # imu/data then comes from ATTITUDE: still watched at its 20 Hz.
    assert keeper.attitude_rate_hz() == 20.0


def test_losing_mavros_while_asking_waits_for_the_reconnect(keeper_module):
    def drop_link(node, message_id):
        if message_id == 27:
            connect(node, False)

    autopilot = Autopilot(on_request=drop_link)
    keeper = keeper_with(keeper_module, RATES, autopilot)
    connect(keeper)
    keeper.step(0.0)
    assert [message for message, _ in autopilot.requests] == [30, 31, 27]
    keeper.step(10.0)
    assert len(autopilot.requests) == 3
    autopilot.on_request = None
    connect(keeper)
    keeper.step(20.0)
    assert autopilot.requests[3:] == list(RATES.items())


def test_it_only_ever_asks_for_message_intervals(keeper_module):
    """Read-only towards the vehicle apart from MAV_CMD_SET_MESSAGE_INTERVAL:
    no parameter writes, modes, arming or setpoints."""
    keeper = keeper_with(keeper_module, RATES, Autopilot())
    assert set(keeper.clients) == {'/mavros/set_message_interval'}
    assert set(keeper.subscriptions) == {'/mavros/state', '/mavros/imu/data'}
    assert keeper.publishers == []
    source = KEEPER.read_text()
    for forbidden in ('create_publisher', 'ParamSet', 'param/set', 'CommandBool',
                      'arming', 'SetMode', 'set_mode', 'cmd/command'):
        assert forbidden not in source


def test_main_reads_the_rates_file_and_steps_the_keeper(keeper_module, tmp_path):
    rates_file = tmp_path / 'rates.yaml'
    rates_file.write_text('message_rates_hz:\n  ATTITUDE: 10\n  VFR_HUD: 5\n')
    loops = iter(range(3))
    keeper_module.rclpy = NS(init=lambda: None, ok=lambda: next(loops, None) is not None,
                             spin_once=lambda node, timeout_sec: None,
                             shutdown=lambda: None)
    nodes = []
    real = keeper_module.MessageRateKeeper

    def make(*args, **kwargs):
        nodes.append(real(*args, **kwargs))
        nodes[-1].step = MagicMock()
        return nodes[-1]

    with patch.object(keeper_module, 'MessageRateKeeper', make):
        keeper_module.main(['--rates', str(rates_file), '--namespace', '/mavros/'])
    [node] = nodes
    assert node.rates == {30: 10.0, 74: 5.0}
    assert '/mavros/state' in node.subscriptions
    assert node.step.call_count == 3
    with pytest.raises(SystemExit):
        keeper_module.parse_args([])


# --- the shared request function (flight_ready) --------------------------------

def test_flight_ready_still_requests_the_aerodrone_rates_through_it(modules):
    ready, _ = modules
    node = ready.NavigationReadiness('/mavros', require_gps=False)
    node.state = NS(connected=True)
    autopilot = Autopilot({33: [False]})
    ready.request_message_interval = autopilot
    assert not node.configure_intervals()
    assert autopilot.requests == [(24, 5.0), (30, 20.0), (31, 20.0), (32, 20.0), (33, 5.0)]
    assert node.configure_intervals()
    assert autopilot.requests[5:] == list(ready.MESSAGE_RATES_HZ.items())
    assert node.intervals_configured


@pytest.mark.parametrize('response,accepted', [
    (NS(success=True), True), (NS(success=False), False), (None, False)])
def test_request_message_interval_reports_whether_the_autopilot_accepted(
        modules, response, accepted):
    ready, _ = modules
    sent, spun = [], []
    client = NS(call_async=lambda request: sent.append(request) or NS(result=lambda: response))
    ready.rclpy = NS(spin_until_future_complete=lambda node, future, timeout_sec:
                     spun.append(timeout_sec))
    ready.MessageInterval = NS(Request=NS)
    assert ready.request_message_interval('node', client, 31, 20.0) is accepted
    assert (sent[0].message_id, sent[0].message_rate) == (31, 20.0)
    assert spun == [5.0]

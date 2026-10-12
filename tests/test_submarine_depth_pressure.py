"""Submarine: /sub0/sensor_measurements/pressure is the water pressure of
ArduSub's external depth sensor, not the air inside the enclosure.

MAVROS 2.14 publishes /mavros/imu/static_pressure from SCALED_PRESSURE only:
ArduPilot's barometer 0, on a Navigator the board's own barometer (the Pi
5 bench Navigator's BARO1_DEVID is a BMP390 on I2C bus 1, 0x76: 1004 hPa at
35 C). ArduSub probes a Bar30 or Bar100 on BARO_EXT_BUS (6 on a Navigator,
its I2C6 header) after it, so the depth sensor is barometer 1, sent as
SCALED_PRESSURE2, which no MAVROS plugin publishes. ArduSub makes the first
water-pressure barometer its primary (its EKF height and depth) and reports
an absolute-pressure sensor in SYS_STATUS only when it has one.

The bridge (mavros_mirror, pressure_source ardusub_depth_sensor) decodes
SCALED_PRESSURE2 from MAVROS's raw MAVLink stream (/uas1/mavlink_source;
MAVROS 2 has no /mavros/mavlink/from) and publishes barometer 0 with a
warning when no depth sensor reports. The simulation keeps barometer 0,
which is a water sensor in ArduSub SITL.
"""
import struct
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

MIRROR = REPO / 'applications/aerostack2/mavros_mirror/mavros_mirror/__init__.py'
CONFIG = REPO / 'systems/submarine/config'
OUTPUT = '/sub0/sensor_measurements/pressure'
BAROMETER0 = '/mavros/imu/static_pressure'
RAW = '/uas1/mavlink_source'
SYS_STATUS = '/mavros/sys_status'
# SYS_STATUS sensors_present of the bench ArduSub 4.7.1 (no depth sensor),
# and with MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE (0x08) set.
NO_DEPTH_SENSOR = 0x1331fc07
DEPTH_SENSOR = NO_DEPTH_SENSOR | 0x08


class FluidPressure:
    def __init__(self, pressure=0.0, frame_id='', stamp=None):
        self.header = NS(stamp=stamp, frame_id=frame_id)
        self.fluid_pressure = pressure
        self.variance = 0.0


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warning(self, text):
        self.lines.append(('warning', text))

    warn = warning


class FakeNode:
    def __init__(self, name):
        if hasattr(self, 'params'):
            return
        self.params, self.publishers, self.callbacks, self.qos = {}, {}, {}, {}
        self.logger = Logger()

    def declare_parameter(self, name, value):
        self.params.setdefault(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        return self.publishers.setdefault(topic, MagicMock(msg_type=msg_type))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback
        self.qos[topic] = qos

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: 'now'))

    def get_logger(self):
        return self.logger


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


@pytest.fixture
def mirror():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('mavros_msgs', 'mavros_msgs.msg'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['sensor_msgs.msg'] = MagicMock(FluidPressure=FluidPressure)
        module = load_script(MIRROR, 'mavros_mirror_depth_review')
    module.time = Clock()
    return module


def make_bridge(module, path, **overrides):
    params = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    params.update(overrides)
    node = module.MavrosMirror.__new__(module.MavrosMirror)
    FakeNode.__init__(node, 'mavros_mirror')
    node.params.update(params)
    module.MavrosMirror.__init__(node)
    return node


def raw_message(payload, msgid=137, sysid=1, compid=1, framing=1, stamp='rx'):
    """mavros_msgs/Mavlink as MAVROS's router publishes it: the payload in
    little-endian uint64 words, `len` bytes used."""
    words = (len(payload) + 7) // 8
    return NS(msgid=msgid, sysid=sysid, compid=compid, framing_status=framing,
              len=len(payload), header=NS(stamp=stamp, frame_id='ep:1000'),
              payload64=list(struct.unpack(f'<{words}Q', payload.ljust(words * 8, b'\0'))))


def scaled_pressure(press_abs_hpa, temperature_cdeg=1850, time_boot_ms=1000):
    """A SCALED_PRESSURE2 payload, trailing zero bytes trimmed (MAVLink 2)."""
    payload = struct.pack('<Iffhh', time_boot_ms, press_abs_hpa, 0.0, temperature_cdeg, 0)
    return payload.rstrip(b'\0')


def published(node):
    return [call.args[0] for call in node.publishers[OUTPUT].publish.call_args_list]


def warnings(node):
    return [text for level, text in node.logger.lines if level == 'warning']


# --- decoding --------------------------------------------------------------------

# Payloads encoded by pymavlink 2.4.50 (MAVLink 2, common.xml), and one
# SCALED_PRESSURE received on the Pi 5 Navigator through MAVROS 2.14's
# /uas1/mavlink_source, which MAVROS published as 100431.98 Pa on
# /mavros/imu/static_pressure. One payload layout for all three messages.
VECTORS = [
    ('40e201000068be44000000003a07', (123456, 1523.25, 0.0, 1850)),
    ('070000000030fa44000080be6aff4101', (7, 2001.5, -0.25, -150)),
    ('6300000000808944', (99, 1100.0, 0.0, 0)),  # trimmed to 8 bytes
]


@pytest.mark.parametrize('payload,expected', VECTORS)
def test_scaled_pressure_payloads_decode(mirror, payload, expected):
    message = raw_message(bytes.fromhex(payload))
    assert mirror.decode_scaled_pressure(mirror.mavlink_payload(message)) == expected


def test_a_scaled_pressure_received_on_the_navigator_decodes_as_mavros_publishes_it(mirror):
    message = NS(len=14, payload64=[4934560322344715162, 14929306320896])
    boot_ms, press_abs, press_diff, temperature = mirror.decode_scaled_pressure(
        mirror.mavlink_payload(message))
    assert (boot_ms, press_diff, temperature) == (1573786, 0.0, 3476)
    assert press_abs * 100.0 == pytest.approx(100431.98, abs=0.005)


# --- the choice of source ----------------------------------------------------------

def selector(mirror, **kwargs):
    return mirror.DepthPressureSelector('SCALED_PRESSURE2', 5.0, BAROMETER0, **kwargs)


def test_the_external_sensor_is_used_while_it_reports(mirror):
    choice = selector(mirror)
    choice.sys_status(DEPTH_SENSOR)
    assert choice.barometer0(0.0) == (None, None)        # waiting for it
    source, (level, text) = choice.external(0.5)
    assert source == 'external' and level == 'info'
    assert 'SCALED_PRESSURE2 (barometer 1)' in text and 'reports a depth sensor' in text
    # Barometer 0 is not published while the external sensor reports.
    for step in range(1, 100):
        assert choice.barometer0(0.5 + 0.1 * step) == ('external', None)
        assert choice.external(0.55 + 0.1 * step) == ('external', None)


def test_the_external_sensor_counts_before_sys_status_arrives(mirror):
    choice = selector(mirror)
    source, (_, text) = choice.external(0.0)
    assert source == 'external' and 'has not reported its sensors yet' in text
    choice.sys_status(DEPTH_SENSOR)
    assert choice.select(0.1)[0] == 'external'


def test_no_depth_sensor_falls_back_to_barometer_0_with_a_warning(mirror):
    """The bench Navigator: no SCALED_PRESSURE2, and ArduSub's SYS_STATUS
    reports no absolute-pressure (depth) sensor."""
    choice = selector(mirror)
    choice.sys_status(NO_DEPTH_SENSOR)
    assert choice.barometer0(0.0) == (None, None)
    assert choice.barometer0(4.9) == (None, None)
    source, (level, text) = choice.barometer0(5.0)
    assert (source, level) == ('barometer0', 'warning')
    assert 'no SCALED_PRESSURE2 (barometer 1) in 5 s' in text
    assert 'reports no depth sensor' in text and 'not water depth' in text
    assert BAROMETER0 in text
    # Warned once, not with every message.
    assert choice.barometer0(5.1) == ('barometer0', None)


def test_a_depth_sensor_appearing_later_takes_over(mirror):
    choice = selector(mirror)
    choice.barometer0(0.0)
    source, (level, text) = choice.barometer0(5.0)
    assert (source, level) == ('barometer0', 'warning') and 'no SYS_STATUS' in text
    choice.sys_status(DEPTH_SENSOR)
    assert choice.external(6.0)[0] == 'external'
    assert choice.barometer0(6.1) == ('external', None)


def test_a_depth_sensor_that_stops_falls_back_with_a_warning(mirror):
    choice = selector(mirror)
    choice.barometer0(0.0)
    choice.external(0.0)
    assert choice.barometer0(5.0) == ('external', None)  # last one 5 s ago
    source, (level, text) = choice.barometer0(5.1)
    assert (source, level) == ('barometer0', 'warning')
    assert 'SCALED_PRESSURE2 (barometer 1) stopped for 5 s' in text
    assert choice.external(9.0)[0] == 'external'           # and back


def test_a_depth_sensor_that_stops_is_not_taken_for_barometer_0(mirror):
    """ArduSub still reports its depth sensor, but SCALED_PRESSURE2 stopped
    (its stream rate reset, the link): barometer 0 is the board's, so warn."""
    choice = selector(mirror)
    choice.sys_status(DEPTH_SENSOR)
    choice.barometer0(0.0)
    choice.external(0.1)
    source, (level, text) = choice.barometer0(5.2)
    assert (source, level) == ('barometer0', 'warning')
    assert 'stopped for 5 s (ArduSub reports a depth sensor)' in text
    assert 'is it' not in text


def test_barometer_0_is_the_depth_sensor_when_ardusub_has_no_other(mirror):
    """ArduSub reports a depth sensor but sends no SCALED_PRESSURE2: its
    only barometer is the water sensor (ArduSub SITL, or a board without a
    barometer of its own). Barometer 0, without a warning."""
    choice = selector(mirror)
    choice.sys_status(DEPTH_SENSOR)
    choice.barometer0(0.0)
    source, (level, text) = choice.barometer0(5.0)
    assert (source, level) == ('barometer0', 'info')
    assert 'so barometer 0 is it' in text


def test_a_second_barometer_that_is_not_a_depth_sensor_is_not_used(mirror):
    """SCALED_PRESSURE2 from a second air barometer: ArduSub's SYS_STATUS
    reports no depth sensor."""
    choice = selector(mirror)
    choice.sys_status(NO_DEPTH_SENSOR)
    assert choice.external(0.0) == (None, None)
    choice.barometer0(0.0)
    source, (level, text) = choice.barometer0(5.0)
    assert (source, level) == ('barometer0', 'warning')
    assert 'arrives, but' in text and 'reports no depth sensor' in text


def test_a_depth_sensor_message_must_be_a_barometer_message(mirror):
    with pytest.raises(ValueError, match='depth_sensor_message'):
        mirror.DepthPressureSelector('SCALED_PRESSURE', 5.0, BAROMETER0)
    assert mirror.DepthPressureSelector('SCALED_PRESSURE3').message_id == 143


# --- the hardware bridge -------------------------------------------------------------

def test_the_hardware_bridge_publishes_the_external_depth_sensor(mirror):
    node = make_bridge(mirror, CONFIG / 'bridge.yaml')
    assert set(node.callbacks) >= {RAW, SYS_STATUS, BAROMETER0}
    assert node.qos[RAW] is mirror.MAVLINK_QOS and node.qos[SYS_STATUS] is mirror.LATCHED_QOS
    node.callbacks[SYS_STATUS](NS(sensors_present=DEPTH_SENSOR))
    node.callbacks[BAROMETER0](FluidPressure(100431.98, 'base_link', 'b0'))
    node.callbacks[RAW](raw_message(scaled_pressure(1215.5), stamp='rx1'))
    [out] = published(node)
    # 1215.5 hPa: a Bar30 about 2 m under fresh water.
    assert out.fluid_pressure == pytest.approx(121550.0)
    assert (out.header.stamp, out.header.frame_id, out.variance) == ('rx1', 'base_link', 0.0)
    mirror.time.now = 0.1
    node.callbacks[BAROMETER0](FluidPressure(100431.98, 'base_link', 'b1'))
    assert len(published(node)) == 1                       # barometer 0 held back
    assert warnings(node) == []


@pytest.mark.parametrize('message', [
    {'msgid': 29},                     # SCALED_PRESSURE: barometer 0
    {'msgid': 143},                    # SCALED_PRESSURE3: ArduSub's water temperature
    {'sysid': 1, 'compid': 194},       # another component on the vehicle
    {'sysid': 255, 'compid': 190},     # a ground station
    {'framing': 2},                    # bad CRC
])
def test_the_bridge_ignores_other_raw_messages(mirror, message):
    node = make_bridge(mirror, CONFIG / 'bridge.yaml')
    node.callbacks[SYS_STATUS](NS(sensors_present=DEPTH_SENSOR))
    node.callbacks[RAW](raw_message(scaled_pressure(1215.5), **message))
    assert published(node) == []
    assert node.depth_selector.last_external is None


def test_the_hardware_bridge_warns_and_publishes_barometer_0_without_a_depth_sensor(mirror):
    """As on the Pi 5 Navigator (ArduSub 4.7.1, no Bar30): after 5 s the
    bridge logged the warning and published barometer 0 at 10 Hz."""
    node = make_bridge(mirror, CONFIG / 'bridge.yaml')
    node.callbacks[SYS_STATUS](NS(sensors_present=NO_DEPTH_SENSOR))
    for step in range(60):
        mirror.time.now = 0.1 * step
        node.callbacks[BAROMETER0](FluidPressure(100431.98, 'base_link', step))
    assert [message.header.stamp for message in published(node)] == list(range(50, 60))
    [warning] = warnings(node)
    assert 'reports no depth sensor' in warning and 'not water depth' in warning


def test_the_bridge_publishes_only_its_own_topics(mirror):
    node = make_bridge(mirror, CONFIG / 'bridge.yaml')
    assert set(node.publishers) == {f'/sub0/sensor_measurements/{name}' for name in
                                    ('gps', 'odom', 'imu', 'battery', 'pressure')}


def test_the_raw_stream_is_the_one_mavros_hands_its_uas(mirror):
    """MAVROS 2's mavros_node names its UAS /uas<tgt_system> (1 in
    apm.launch, ArduSub's SYSID_THISMAV) and its router publishes every
    MAVLink message to it on <uas>/mavlink_source."""
    params = yaml.safe_load((CONFIG / 'bridge.yaml').read_text())['/**']['ros__parameters']
    assert params['mavlink_source_topic'] == f"/uas{params['autopilot_system_id']}/mavlink_source"
    assert params['sys_status_topic'] == f"{params['mavros_namespace']}/sys_status"
    assert params['pressure_topic'] == f"{params['mavros_namespace']}/imu/static_pressure"


@pytest.mark.parametrize('value', ['scaled_pressure2', 'static'])
def test_unknown_pressure_sources_are_refused(mirror, value):
    with pytest.raises(ValueError, match='pressure_source'):
        make_bridge(mirror, CONFIG / 'bridge.yaml', pressure_source=value)


# --- unchanged elsewhere --------------------------------------------------------------

def test_the_simulation_bridge_still_mirrors_mavros_pressure(mirror):
    """ArduSub SITL's barometers are all water sensors (AP_Baro_SITL) and
    barometer 0 is its depth sensor; bridge_sim.yaml keeps the MAVROS
    topic and subscribes to no raw MAVLink."""
    node = make_bridge(mirror, CONFIG / 'bridge_sim.yaml')
    assert RAW not in node.callbacks and SYS_STATUS not in node.callbacks
    assert node.depth_selector is None
    message = FluidPressure(111325.0, 'base_link', 'sim')
    node.callbacks[BAROMETER0](message)
    node.publishers[OUTPUT].publish.assert_called_once_with(message)


@pytest.mark.parametrize('bridge', ['systems/boat/config/bridge.yaml',
                                    'systems/airplane/config/mavros_bridge.yaml'])
def test_other_platforms_subscribe_to_no_raw_mavlink(mirror, bridge):
    node = make_bridge(mirror, REPO / bridge)
    assert RAW not in node.callbacks and SYS_STATUS not in node.callbacks
    assert not any(topic.endswith('/pressure') for topic in node.publishers)


def test_the_hardware_composition_runs_the_bridge_with_bridge_yaml():
    stack = yaml.safe_load((REPO / 'compositions/submarine_hardware.yaml').read_text())
    bridge = stack['services']['submarine_bridge']
    assert '--params-file /workspace/config/bridge.yaml' in ' '.join(bridge['command'].split())
    assert '../systems/submarine/config:/workspace/config:ro' in bridge['volumes']
    sim = yaml.safe_load((REPO / 'compositions/submarine_sim.yaml').read_text())
    assert 'bridge_sim.yaml' in sim['services']['submarine_bridge']['command']


def test_the_readme_says_where_the_pressure_comes_from():
    text = (REPO / 'systems/submarine/README.md').read_text()
    section = text.split('## Depth pressure', 1)[1].split('\n## ', 1)[0]
    for needed in ('SCALED_PRESSURE2', 'pressure_source', 'BARO_EXT_BUS',
                   '/uas1/mavlink_source', 'warning'):
        assert needed in section, needed

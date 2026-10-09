"""ArduPilot navigation bridge: shared EKF origin relay, GPS_INPUT encoding of
accepted corrections, EKF source-set hysteresis, and the flight-controller
parameter check that gates forwarding."""
import calendar
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


class FakeNode:
    def __init__(self, name):
        if hasattr(self, 'params'):
            return
        self.params = {}
        self.publishers = {}
        self.callbacks = {}
        self.now_ns = 1_760_000_000 * 1_000_000_000
        self.clients = {}
        self.timers = []
        self.logger = MagicMock()

    def declare_parameter(self, name, value):
        self.params.setdefault(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        return self.publishers.setdefault(topic, MagicMock())

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback

    def create_client(self, srv_type, name):
        return self.clients.setdefault(name, MagicMock())

    def create_timer(self, period, callback):
        self.timers.append(callback)
        return MagicMock()

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now_ns))

    def get_logger(self):
        return self.logger


class NavSatFix:
    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2


class Message:
    def __init__(self, **fields):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
        self.__dict__.update(fields)


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('geographic_msgs', 'geographic_msgs.msg', 'mavros_msgs',
                     'mavros_msgs.msg', 'mavros_msgs.srv', 'rcl_interfaces',
                     'rcl_interfaces.msg', 'rcl_interfaces.srv'):
            sys.modules[name] = MagicMock()
        sys.modules['rcl_interfaces.msg'].ParameterValue = lambda **kw: NS(**kw)
        sys.modules['rcl_interfaces.msg'].ParameterType = NS(
            PARAMETER_NOT_SET=0, PARAMETER_INTEGER=2, PARAMETER_DOUBLE=3)
        sys.modules['rcl_interfaces.srv'].GetParameters = NS(Request=Message)
        sys.modules['mavros_msgs.srv'].ParamSetV2 = NS(Request=Message)
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['sensor_msgs.msg'] = NS(NavSatFix=NavSatFix)
        sys.modules['std_msgs.msg'] = NS(Bool=lambda data=False: NS(data=data))
        sys.modules['geographic_msgs.msg'].GeoPointStamped = Message
        sys.modules['mavros_msgs.msg'].GPSINPUT = Message
        sys.modules['mavros_msgs.srv'].CommandLong = NS(Request=Message)
        yield load_script(REPO / 'applications/aerostack2/scripts/ardupilot_navigation_bridge.py',
                          'ardupilot_navigation_bridge_review')


def make_bridge(module, **params):
    node = module.ArduPilotNavigationBridge.__new__(module.ArduPilotNavigationBridge)
    FakeNode.__init__(node, 'bridge')
    node.params.update(params)
    module.ArduPilotNavigationBridge.__init__(node)
    return node


def fix(lat=37.9, lon=-122.6, variance=64.0, stamp_s=1_760_000_000.0,
        covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN):
    ns = int(stamp_s * 1e9)
    return NS(header=NS(stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9)),
              latitude=lat, longitude=lon, altitude=float('nan'),
              position_covariance=[variance, 0, 0, 0, variance, 0, 0, 0, 0],
              position_covariance_type=covariance_type)


def test_gps_week_and_milliseconds_include_leap_seconds(module):
    unix = calendar.timegm((2026, 10, 9, 12, 0, 0))
    week, week_ms = module.gps_week_ms(unix * 10**9 + 250_000_000)
    gps_seconds = unix - calendar.timegm((1980, 1, 6, 0, 0, 0)) + 18
    assert week == gps_seconds // 604800
    assert week_ms == (gps_seconds % 604800) * 1000 + 250
    # 2026-10-09 is a Friday: 5 days + 12 h + 18 s into the GPS week.
    assert week_ms == ((5 * 24 + 12) * 3600 + 18) * 1000 + 250


def test_gps_input_carries_honest_horizontal_accuracy_only(module):
    fields = module.gps_input_fields(fix(variance=144.0), gps_id=1, satellites=6)
    assert fields['horiz_accuracy'] == pytest.approx(12.0)
    assert fields['fix_type'] == 3 and fields['gps_id'] == 1
    assert fields['lat'] == 379000000 and fields['lon'] == -1226000000
    flags = fields['ignore_flags']
    for absent in (module.IGNORE_ALT, module.IGNORE_VEL_HORIZ, module.IGNORE_VEL_VERT,
                   module.IGNORE_SPEED_ACCURACY, module.IGNORE_VERTICAL_ACCURACY):
        assert flags & absent
    assert not flags & module.IGNORE_HORIZONTAL_ACCURACY
    assert fields['hdop'] == pytest.approx(12.0 / module.NOMINAL_UERE_M)


@pytest.mark.parametrize('bad', [
    dict(covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN),
    dict(variance=float('nan')), dict(variance=0.0),
    dict(lat=float('nan')), dict(lon=200.0)])
def test_gps_input_refuses_fixes_without_honest_accuracy(module, bad):
    assert module.gps_input_fields(fix(**bad), gps_id=1, satellites=6) is None


def test_source_set_hysteresis(module):
    switch = module.SourceSetSwitch(loss_s=1.0, recovery_s=10.0)
    # Acquiring the first fix on the ground is not GPS denial.
    assert switch.update(False, 0.0) is None
    assert switch.update(False, 5.0) is None
    assert switch.update(True, 6.0) is None
    assert switch.update(False, 7.0) is None
    assert switch.update(False, 7.5) is None
    assert switch.update(False, 8.1) == 2
    switch.current = 2
    assert switch.update(True, 9.0) is None
    assert switch.update(False, 9.5) is None  # flicker restarts recovery
    assert switch.update(True, 10.0) is None
    assert switch.update(True, 19.0) is None
    assert switch.update(True, 20.1) == 1


def test_source_set_switches_when_armed_without_ever_seeing_a_fix(module):
    switch = module.SourceSetSwitch(loss_s=1.0, recovery_s=10.0)
    switch.armed = True
    assert switch.update(False, 0.0) is None
    assert switch.update(False, 1.5) == 2


def test_origin_is_relayed_once_and_invalid_origins_are_ignored(module):
    bridge = make_bridge(module)
    publisher = bridge.publishers['/navigation/origin']
    callback = bridge.callbacks['/mavros/global_position/gp_origin']
    for lat, lon, alt in ((0.0, 0.0, 0.0), (float('nan'), -122.0, 0.0), (95.0, 0.0, 0.0)):
        callback(Message(position=NS(latitude=lat, longitude=lon, altitude=alt)))
    publisher.publish.assert_not_called()
    origin = Message(position=NS(latitude=37.9234, longitude=-122.5967, altitude=749.3))
    callback(origin)
    callback(origin)
    publisher.publish.assert_called_once()
    assert publisher.publish.call_args.args[0].position.altitude == 749.3


def test_origin_is_requested_until_received(module):
    bridge = make_bridge(module)
    client = bridge.clients['/mavros/cmd/command']
    client.service_is_ready.return_value = True
    request_origin = bridge.timers[0]
    request_origin()
    request = client.call_async.call_args.args[0]
    assert request.command == 512 and request.param1 == 49.0
    bridge.callbacks['/mavros/global_position/gp_origin'](
        Message(position=NS(latitude=37.9, longitude=-122.6, altitude=10.0)))
    client.call_async.reset_mock()
    request_origin()
    client.call_async.assert_not_called()


def test_corrections_are_not_forwarded_unless_enabled(module):
    bridge = make_bridge(module)
    assert '/navigation/correction_fix' not in bridge.callbacks
    active = bridge.publishers['/navigation/autopilot_corrections_active']
    assert active.publish.call_args.args[0].data is False


GPS_INPUT_PARAMS = REPO / 'systems/aerodrone/config/arducopter_gps_input.params'


def gps_input_param_file():
    values = {}
    for line in GPS_INPUT_PARAMS.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            name, value = line.split(',')
            values[name.strip()] = float(value)
    return values


def answer_fc_check(bridge, as_double=False, **overrides):
    """Run the bridge's FC check once; MAVROS answers with the GPS_INPUT
    parameter file plus overrides (None = MAVROS does not have it)."""
    values = dict(gps_input_param_file(), **overrides)
    client = bridge.clients['/mavros/param/get_parameters']
    client.service_is_ready.return_value = True
    client.call_async.reset_mock()
    bridge.timers[1]()
    request = client.call_async.call_args.args[0]

    def encode(value):
        if value is None:
            return NS(type=0, integer_value=0, double_value=0.0)
        if as_double:
            return NS(type=3, integer_value=0, double_value=float(value))
        return NS(type=2, integer_value=int(value), double_value=0.0)

    future = client.call_async.return_value
    future.result.return_value = NS(values=[encode(values.get(name))
                                            for name in request.names])
    future.add_done_callback.call_args.args[0](future)
    return request


def active_flags(bridge):
    publisher = bridge.publishers['/navigation/autopilot_corrections_active']
    return [call.args[0].data for call in publisher.publish.call_args_list]


def test_required_fc_params_match_the_gps_input_parameter_file(module):
    required = module.required_fc_params()
    assert set(required) == set(gps_input_param_file())
    for name, value in gps_input_param_file().items():
        if name == 'GPS2_TYPE':
            # Managed by the bridge at runtime: 0 on the ground, 14 in flight.
            assert value == 0 and required[name] == (0, 14)
        else:
            assert required[name] == (value,), name
    assert 'EK3_SRC3_POSXY' in module.required_fc_params(denied_set=3)


def test_check_fc_params_separates_missing_from_wrong(module):
    required = {'A': (1,), 'B': (0, 14), 'C': (0,)}
    assert module.check_fc_params(required, {'A': 1, 'B': 14.0, 'C': 0}) == ([], [])
    missing, wrong = module.check_fc_params(required, {'A': 2, 'B': 1, 'C': None})
    assert missing == ['C']
    assert wrong == ['A=2 (needs 1)', 'B=1 (needs 0 or 14)']
    _, wrong = module.check_fc_params(required, {'A': float('nan'), 'B': 0, 'C': 0})
    assert wrong == ['A=nan (needs 1)']


def test_disabled_bridge_never_reads_or_writes_fc_parameters(module):
    bridge = make_bridge(module)
    assert len(bridge.timers) == 1
    assert '/mavros/param/get_parameters' not in bridge.clients
    assert '/mavros/param/set' not in bridge.clients
    assert active_flags(bridge) == [False]


def test_forwarding_arms_only_after_fc_parameters_are_verified(module):
    bridge = make_bridge(module, gps_input_enabled=True)
    assert active_flags(bridge) == [False]
    out = bridge.publishers['/mavros/gps_input/gps_input']
    params = bridge.clients['/mavros/param/set']
    command = bridge.clients['/mavros/cmd/command']
    params.service_is_ready.return_value = True
    command.service_is_ready.return_value = True
    now = bridge.now_ns / 1e9
    # Before verification: no GPS_INPUT, no GPS2_TYPE write, no source switch.
    bridge.callbacks['/navigation/correction_fix'](fix(stamp_s=now - 0.2))
    bridge.callbacks['/mavros/state'](Message(connected=True, armed=False))
    bridge.callbacks['/mavros/gpsstatus/gps1/raw'](Message(fix_type=0))
    out.publish.assert_not_called()
    params.call_async.assert_not_called()
    command.call_async.assert_not_called()
    # MAVROS has not fetched the parameters yet: still waiting, not an error.
    request = answer_fc_check(bridge, EK3_SRC_OPTIONS=None)
    assert set(request.names) == set(module.required_fc_params())
    assert active_flags(bridge) == [False]
    bridge.logger.error.assert_not_called()
    answer_fc_check(bridge, as_double=True)
    assert active_flags(bridge) == [False, True]
    bridge.callbacks['/navigation/correction_fix'](fix(stamp_s=now - 0.2))
    out.publish.assert_called_once()


@pytest.mark.parametrize('wrong', [
    dict(EK3_SRC_OPTIONS=1),                   # ArduPilot default: fuse all velocities
    dict(EK3_SRC2_POSXY=0, EK3_SRC2_YAW=0),    # source set 2 left at defaults
    dict(GPS2_TYPE=1),                         # a real second receiver
    dict(GPS_AUTO_SWITCH=1)])
def test_fc_without_the_gps_input_configuration_never_forwards(module, wrong):
    """Hardware safety: with the FC parameters wrong the bridge logs which,
    forwards nothing and writes nothing to the FC (no EKF source parameters,
    no GPS2_TYPE, no source-set switch), until the FC is fixed."""
    bridge = make_bridge(module, gps_input_enabled=True)
    params = bridge.clients['/mavros/param/set']
    command = bridge.clients['/mavros/cmd/command']
    params.service_is_ready.return_value = True
    command.service_is_ready.return_value = True
    answer_fc_check(bridge, **wrong)
    message = bridge.logger.error.call_args.args[0]
    assert 'NOT active' in message and 'arducopter_gps_input.params' in message
    for name in wrong:
        assert name in message
    state = bridge.callbacks['/mavros/state']
    gps1 = bridge.callbacks['/mavros/gpsstatus/gps1/raw']
    state(Message(connected=True, armed=False))
    state(Message(connected=True, armed=True))
    t0 = bridge.now_ns
    gps1(Message(fix_type=3))
    bridge.now_ns = t0 + 3 * 10**9
    gps1(Message(fix_type=0))
    bridge.callbacks['/navigation/correction_fix'](fix(stamp_s=bridge.now_ns / 1e9 - 0.2))
    state(Message(connected=True, armed=False))
    params.call_async.assert_not_called()
    command.call_async.assert_not_called()
    bridge.publishers['/mavros/gps_input/gps_input'].publish.assert_not_called()
    assert active_flags(bridge) == [False]
    # The same mismatch is not re-logged every check; loading the file on
    # the autopilot arms forwarding without a restart.
    answer_fc_check(bridge, **wrong)
    assert bridge.logger.error.call_count == 1
    answer_fc_check(bridge)
    assert active_flags(bridge) == [False, True]


def test_gps2_left_configured_by_a_stopped_bridge_is_reset_on_the_ground(module):
    bridge = make_bridge(module, gps_input_enabled=True)
    params = bridge.clients['/mavros/param/set']
    params.service_is_ready.return_value = True
    answer_fc_check(bridge, GPS2_TYPE=14)
    assert active_flags(bridge) == [False, True]
    bridge.callbacks['/mavros/state'](Message(connected=True, armed=False))
    request = params.call_async.call_args.args[0]
    assert (request.param_id, request.value.integer_value) == ('GPS2_TYPE', 0)


def test_enabled_bridge_forwards_fresh_corrections_only(module):
    bridge = make_bridge(module, gps_input_enabled=True)
    answer_fc_check(bridge)
    assert bridge.publishers['/navigation/autopilot_corrections_active'] \
        .publish.call_args.args[0].data is True
    out = bridge.publishers['/mavros/gps_input/gps_input']
    callback = bridge.callbacks['/navigation/correction_fix']
    now = bridge.now_ns / 1e9
    callback(fix(stamp_s=now - 3.0))
    out.publish.assert_not_called()
    callback(fix(stamp_s=now - 0.2, variance=100.0))
    sent = out.publish.call_args.args[0]
    assert sent.horiz_accuracy == pytest.approx(10.0)
    assert sent.gps_id == 1 and sent.satellites_visible == 6
    assert not math.isnan(sent.alt)


def _requests(client):
    return [call.args[0] for call in client.call_async.call_args_list]


def test_gps2_exists_only_while_gps1_is_lost_in_flight(module):
    """GPS2_TYPE=14 with no fix blocks arming (GPS 2: Bad fix), so the bridge
    configures GPS 2 only when the primary GPS is lost and removes it on the
    ground; the EKF switches to the position-only source set meanwhile."""
    bridge = make_bridge(module, gps_input_enabled=True)
    answer_fc_check(bridge)
    params = bridge.clients['/mavros/param/set']
    command = bridge.clients['/mavros/cmd/command']
    params.service_is_ready.return_value = True
    command.service_is_ready.return_value = True
    state = bridge.callbacks['/mavros/state']
    gps1 = bridge.callbacks['/mavros/gpsstatus/gps1/raw']
    state(Message(connected=True, armed=False))
    assert [(r.param_id, r.value.integer_value) for r in _requests(params)] == [('GPS2_TYPE', 0)]
    params.call_async.return_value.add_done_callback.call_args.args[0](
        NS(result=lambda: NS(success=True)))
    state(Message(connected=True, armed=True))
    t0 = bridge.now_ns
    gps1(Message(fix_type=3))
    bridge.now_ns = t0 + 2 * 10**9
    gps1(Message(fix_type=1))
    bridge.now_ns = t0 + 4 * 10**9
    gps1(Message(fix_type=1))
    assert _requests(params)[-1].value.integer_value == 14
    assert _requests(command)[-1].command == 42007
    assert _requests(command)[-1].param1 == 2.0
    # Recovery: back to the nominal source set after gps_recovery_s.
    bridge.now_ns = t0 + 5 * 10**9
    gps1(Message(fix_type=3))
    bridge.now_ns = t0 + 16 * 10**9
    gps1(Message(fix_type=3))
    assert _requests(command)[-1].param1 == 1.0
    # Landing and disarming removes GPS 2 again.
    state(Message(connected=True, armed=False))
    assert _requests(params)[-1].value.integer_value == 0

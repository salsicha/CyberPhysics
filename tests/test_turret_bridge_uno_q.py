"""Turret bridge over the UNO Q's arduino-router (transport uno_q_bridge).

The bridge talks msgpack-RPC to a fake arduino-router on a real Unix socket;
the fake also plays the turret firmware on the MCU (turret/set, turret/stop,
turret/get_position, and the turret/pos and turret/err notifications). The
scenarios mirror test_turret_bridge.py (serial transport): no fabricated joint
states, e-stop and link gating, plus RPC-specific failures.
"""
import math
import select
import shutil
import socket
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, install_ros_stubs, load_script

msgpack = pytest.importorskip('msgpack')


@pytest.fixture(scope='module')
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors'):
            sys.modules[name] = MagicMock()
        sys.modules['serial'] = None
        module = load_script(REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py',
                             'turret_arduino_bridge_uno_q')
        module.Bool = lambda data: NS(data=data)
        module.JointState = lambda: NS(header=NS(stamp=None), name=[], position=[])
        yield module


class FakeRouter:
    """arduino-router with turret_firmware behind it, driven synchronously."""

    def __init__(self, path):
        self.path = path
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(path)
        self.server.listen(4)
        self.conn = None
        self.unpacker = None
        self.requests = []       # (method, params) received
        self.responses = []      # responses received from the bridge
        self.registered = set()
        self.foreign_routes = set()  # owned by another client
        self.mcu = 'running'     # 'running', 'absent' (sketch not running), 'silent'
        self.position = [0.1, -0.2]

    def accept(self):
        readable, _, _ = select.select([self.server], [], [], 1.0)
        assert readable, 'the bridge did not connect'
        self.conn, _ = self.server.accept()
        self.unpacker = msgpack.Unpacker(raw=False)
        self.registered = set()

    def send(self, message):
        try:
            self.conn.sendall(msgpack.packb(message))
        except (BrokenPipeError, ConnectionResetError):
            pass  # the bridge has gone; the real router just logs it

    def pump(self, quiet_s=0.05):
        """Answer everything the bridge has sent, as the router and the MCU would."""
        while True:
            readable, _, _ = select.select([self.conn], [], [], quiet_s)
            if not readable:
                return
            data = self.conn.recv(4096)
            if not data:
                return
            self.unpacker.feed(data)
            for message in self.unpacker:
                self.handle(message)

    def handle(self, message):
        if message[0] == 1:
            self.responses.append(message)
            return
        assert message[0] == 0, message
        _, msgid, method, params = message
        self.requests.append((method, params))
        if method == '$/register':
            if params[0] in self.foreign_routes:
                self.send([1, msgid, [5, f'route already exists: {params[0]}'], None])
            else:
                self.registered.add(params[0])
                self.send([1, msgid, None, True])
        elif method.startswith('turret/'):
            if self.mcu == 'absent':
                self.send([1, msgid, [2, f'method {method} not available'], None])
            elif self.mcu == 'running':
                if method == 'turret/set':
                    self.send([1, msgid, None, all(math.isfinite(v) for v in params)])
                elif method == 'turret/stop':
                    self.send([1, msgid, None, True])
                elif method == 'turret/get_position':
                    self.send([1, msgid, None, self.position])
                else:
                    self.send([1, msgid, [0xFE, method], None])
        else:
            self.send([1, msgid, [2, f'method {method} not available'], None])

    def notify(self, method, params):
        """Firmware notification; the router drops it unless someone registered."""
        if method in self.registered:
            self.send([2, method, params])

    def methods(self, prefix='turret/'):
        return [m for m, _ in self.requests if m.startswith(prefix)]

    def drop_connection(self):
        self.conn.close()
        self.conn = None

    def close(self):
        if self.conn is not None:
            self.conn.close()
        self.server.close()


@pytest.fixture
def router():
    directory = tempfile.mkdtemp(prefix='turret-', dir='/tmp')
    fake = FakeRouter(str(Path(directory) / 'router.sock'))
    yield fake
    fake.close()
    shutil.rmtree(directory, ignore_errors=True)


@pytest.fixture
def bridge(module, router):
    node = module.TurretArduinoBridge.__new__(module.TurretArduinoBridge)
    node.get_logger = MagicMock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp'))
    node.transport = 'uno_q_bridge'
    node.router_socket = router.path
    node.rpc_timeout_s = 0.5
    node.router = None
    node.mcu_ready = False
    node.registered = set()
    node.last_probe_time = float('-inf')
    node.probe_pending = False
    node.serial_port = '/dev/ttyACM0'
    node.baud_rate = 115200
    node.dry_run = False
    node.pan_min_rad, node.pan_max_rad = -1.5708, 1.5708
    node.tilt_min_rad, node.tilt_max_rad = -0.65, 0.75
    node.serial_handle = None
    node.serial_ready_time = 0.0
    node.last_connect_attempt = float('-inf')
    node.reconnect_interval_s = 2.0
    node.connect_settle_s = 2.0
    node.last_error = ''
    node.last_write_time = node.last_feedback_time = None
    node.rx_buffer = b''
    node.estop_engaged = False
    node.stop_sent = False
    node.dropped_commands = 0
    node.last_ready = None
    node.joint_state_pub = MagicMock()
    node.ready_pub = MagicMock()
    node.diagnostics_pub = MagicMock()
    yield node
    if node.router is not None:
        node.router.close()


def connect(bridge, router):
    """Connect, let the fake router and MCU answer, and process the answers."""
    bridge._poll()
    router.accept()
    router.pump()
    bridge._poll()


def command(pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt])


def last_ready(bridge):
    return bridge.ready_pub.publish.call_args[0][0].data


def test_parse_position_params(module):
    assert module.parse_position_params([0.25, -0.1]) == (0.25, -0.1)
    assert module.parse_position_params((1, 0)) == (1.0, 0.0)
    for bad in ([], [0.1], [0.1, 0.2, 0.3], [math.nan, 0.0], [0.0, math.inf],
                ['0.1', 0.2], [True, 0.0], None, 'POS', {'pan_joint': 0.1}):
        assert module.parse_position_params(bad) is None


def test_connects_registers_and_becomes_ready(bridge, router):
    assert not bridge.actuator_ready()
    bridge._poll()
    router.accept()
    assert not bridge.actuator_ready()  # MCU has not answered yet
    assert bridge._settling()
    router.pump()
    assert router.registered == {'turret/pos', 'turret/err'}
    assert router.methods() == ['turret/get_position']
    bridge._poll()
    assert bridge.mcu_ready
    assert bridge.actuator_ready()
    assert last_ready(bridge) is True
    assert bridge.last_error == ''
    bridge.joint_state_pub.publish.assert_not_called()  # the probe is not a measurement


def test_set_is_sent_as_rpc_and_never_echoed(bridge, router):
    connect(bridge, router)
    bridge._joint_command(command(0.5, 2.0))
    router.pump()
    assert router.requests[-1] == ('turret/set', [0.5, 0.75])  # clamped like serial
    assert bridge.last_write_time is not None
    bridge._poll()
    bridge.joint_state_pub.publish.assert_not_called()
    assert bridge.actuator_ready()


def test_non_finite_command_is_rejected(bridge, router):
    connect(bridge, router)
    for value in (math.nan, math.inf):
        bridge._joint_command(command(value, 0.0))
        bridge._joint_command(command(0.0, value))
    router.pump()
    assert 'turret/set' not in router.methods()
    assert 'non-finite' in bridge.last_error


def test_measured_positions_and_errors_come_from_notifications(bridge, router):
    connect(bridge, router)
    router.notify('turret/pos', [0.1, -0.2])
    router.notify('turret/pos', [math.nan, 0.0])
    router.notify('turret/pos', [0.3])
    router.notify('turret/err', ['servo 13 (tilt_joint) timeout'])
    bridge._poll()
    assert bridge.joint_state_pub.publish.call_count == 1
    state = bridge.joint_state_pub.publish.call_args[0][0]
    assert state.name == ['pan_joint', 'tilt_joint']
    assert list(state.position) == [0.1, -0.2]
    assert bridge.last_error == 'firmware: servo 13 (tilt_joint) timeout'
    assert bridge.last_feedback_time is not None
    # A delivered command clears it, as a serial write does
    bridge._joint_command(command(0.0, 0.0))
    assert bridge.last_error == ''


def test_firmware_call_is_answered(bridge, router):
    connect(bridge, router)
    router.send([0, 77, 'turret/pos', [0.3, 0.4]])
    bridge._poll()
    assert list(bridge.joint_state_pub.publish.call_args[0][0].position) == [0.3, 0.4]
    router.pump()
    assert router.responses[-1] == [1, 77, None, True]


def test_emergency_stop_sends_stop_and_blocks_commands(bridge, router):
    connect(bridge, router)
    bridge._emergency_stop(NS(data=True))
    assert last_ready(bridge) is False
    bridge._joint_command(command(0.1, 0.1))
    bridge._poll()  # STOP is not repeated
    router.pump()
    assert router.methods() == ['turret/get_position', 'turret/stop']
    assert bridge.dropped_commands == 1
    bridge._emergency_stop(NS(data=False))
    assert last_ready(bridge) is True
    bridge._joint_command(command(0.1, 0.1))
    router.pump()
    assert router.requests[-1] == ('turret/set', [0.1, 0.1])


def test_stop_sent_once_mcu_answers_when_release_required(bridge, router):
    bridge.estop_engaged = True  # require_estop_release default
    bridge._poll()
    router.accept()
    bridge._poll()  # MCU not confirmed yet: nothing but the handshake
    router.pump()
    assert 'turret/stop' not in router.methods()
    bridge._poll()  # answers processed: MCU ready, STOP goes out
    bridge._poll()
    router.pump()
    assert router.methods().count('turret/stop') == 1


def test_sketch_not_running_is_not_ready(bridge, router):
    router.mcu = 'absent'
    connect(bridge, router)
    assert not bridge.actuator_ready()
    assert 'turret_firmware' in bridge.last_error
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.dropped_commands == 1
    # The sketch starts: the next probe finds it
    router.mcu = 'running'
    bridge.last_probe_time = float('-inf')
    bridge._poll()
    router.pump()
    bridge._poll()
    assert bridge.actuator_ready()
    assert bridge.last_error == ''


def test_unanswered_rpc_marks_mcu_not_ready(bridge, router):
    connect(bridge, router)
    bridge.router.timeout_s = 0.05
    router.mcu = 'silent'
    bridge._joint_command(command(0.2, 0.0))
    router.pump()
    time.sleep(0.06)
    bridge._poll()
    assert not bridge.mcu_ready
    assert not bridge.actuator_ready()
    assert last_ready(bridge) is False
    assert 'no answer to turret/set' in bridge.last_error
    bridge._joint_command(command(0.3, 0.0))
    assert bridge.dropped_commands == 1


def test_router_restart_reconnects(bridge, router):
    connect(bridge, router)
    router.drop_connection()
    bridge._poll()
    assert not bridge._connected()
    assert not bridge.actuator_ready()
    assert 'arduino-router' in bridge.last_error
    bridge._poll()  # within reconnect_interval_s: no new attempt
    assert not bridge._connected()
    bridge.last_connect_attempt = float('-inf')
    connect(bridge, router)
    assert bridge.actuator_ready()


def test_missing_socket_reports_error(bridge, router):
    bridge.router_socket = router.path + '.missing'
    bridge._poll()
    assert not bridge._connected()
    assert bridge.router_socket in bridge.last_error
    assert not bridge.actuator_ready()


def test_second_bridge_is_never_ready(bridge, router):
    router.foreign_routes = {'turret/pos'}
    connect(bridge, router)
    assert bridge.mcu_ready
    assert not bridge.actuator_ready()
    assert 'another arduino-router client' in bridge.last_error
    bridge._joint_command(command(0.1, 0.1))
    router.pump()
    assert 'turret/set' not in router.methods()


def test_stop_on_shutdown(bridge, router):
    connect(bridge, router)
    assert bridge._send_stop()
    bridge._close_router()
    router.pump()
    assert router.methods()[-1] == 'turret/stop'
    assert not bridge._connected()


def test_dry_run_is_never_ready(bridge, router):
    bridge.dry_run = True
    bridge._poll()
    assert bridge.router is None
    assert not bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.dropped_commands == 1


def test_diagnostics_name_the_router(bridge, router, module):
    module.KeyValue = lambda key, value: NS(key=key, value=value)
    module.DiagnosticStatus = MagicMock(OK=0, WARN=1, ERROR=2)
    module.DiagnosticArray = lambda: NS(header=NS(stamp=None), status=[])
    connect(bridge, router)
    bridge._publish_diagnostics()
    status = bridge.diagnostics_pub.publish.call_args[0][0].status[0]
    values = {kv.key: kv.value for kv in status.values}
    assert values['transport'] == 'uno_q_bridge'
    assert values['router_socket'] == router.path
    assert values['ready'] == 'true'
    assert status.hardware_id == router.path
    assert status.message == 'UNO Q MCU connected, no position feedback from firmware'


def test_link_matches_responses_and_expires_requests(module, router):
    link = module.RouterBridgeLink(router.path, 0.05)
    link.connect()
    router.accept()
    results = {}
    link.request('a', [1], lambda error, result: results.setdefault('a', (error, result)))
    link.request('b', [], lambda error, result: results.setdefault('b', (error, result)))
    link.request('c', [], lambda error, result: results.setdefault('c', (error, result)))
    router.unpacker.feed(router.conn.recv(4096))
    requests = list(router.unpacker)
    assert [r[2] for r in requests] == ['a', 'b', 'c']
    # Answer out of order; leave c unanswered
    router.send([1, requests[1][1], None, 'B'])
    router.send([1, requests[0][1], [3, 'failed to send request'], None])
    router.send([2, 'turret/pos', [0.5, 0.25]])
    incoming = link.poll()
    assert results == {'a': ([3, 'failed to send request'], None), 'b': (None, 'B')}
    assert incoming == [(2, 'turret/pos', [0.5, 0.25], None)]
    time.sleep(0.06)
    link.poll()
    assert results['c'][0].startswith('no answer to c')
    link.request('d', [], lambda error, result: results.setdefault('d', (error, result)))
    link.close()
    assert results['d'][0] == 'connection to arduino-router closed'
    assert not link.connected
    assert module.format_rpc_error([2, 'method x not available']) == \
        'method x not available (code 2)'
    assert module.rpc_error_code([5, 'route already exists: x']) == 5
    assert module.rpc_error_code('text') is None

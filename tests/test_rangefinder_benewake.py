"""Benewake TFmini-S / TF-Luna rangefinder: 9-byte frame parser, REP 117
range values, and the ROS node's publishing, reconnect and stale checks.

The node runs with ROS modules stubbed and an injected fake serial port, so
neither rclpy nor pyserial is needed.
"""
import math
import sys
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/rangefinder/scripts'


@pytest.fixture
def bw(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPTS))
    return load_script(SCRIPTS / 'benewake.py', 'benewake')


# Frame parsing ------------------------------------------------------------

def test_datasheet_example_frame(bw):
    # 0x59 0x59, dist 0x0102 = 258 cm, strength 0x03E8 = 1000,
    # temp 0x0918 = 2328 -> 2328/8 - 256 = 35 degC
    body = bytes([0x59, 0x59, 0x02, 0x01, 0xE8, 0x03, 0x18, 0x09])
    data = body + bytes([sum(body) & 0xFF])
    frame = bw.decode_frame(data)
    assert frame == bw.Frame(distance_cm=258, strength=1000, temperature_c=35.0)
    assert bw.encode_frame(258, 1000, 35.0) == data


def test_checksum_is_low_byte_of_first_eight(bw):
    data = bw.encode_frame(1200, 65535, 60.0)
    assert data[8] == sum(data[:8]) % 256
    assert sum(data[:8]) > 255  # the carry is discarded


def test_good_frames_back_to_back(bw):
    parser = bw.FrameParser()
    stream = b''.join(bw.encode_frame(d, 500 + d) for d in (100, 250, 1199))
    frames = parser.feed(stream)
    assert [f.distance_cm for f in frames] == [100, 250, 1199]
    assert [f.strength for f in frames] == [600, 750, 1699]
    assert parser.checksum_errors == 0 and parser.discarded_bytes == 0


def test_bad_checksum_is_rejected(bw):
    parser = bw.FrameParser()
    bad = bytearray(bw.encode_frame(300, 1000))
    bad[8] ^= 0xFF
    assert bw.decode_frame(bytes(bad)) is None
    assert parser.feed(bytes(bad)) == []
    assert parser.checksum_errors == 1
    # The next good frame still decodes.
    assert [f.distance_cm for f in parser.feed(bw.encode_frame(301, 1000))] == [301]


def test_corrupted_payload_is_rejected(bw):
    data = bytearray(bw.encode_frame(300, 1000))
    data[2] ^= 0x01  # one flipped distance bit
    assert bw.FrameParser().feed(bytes(data)) == []


def test_frames_split_across_reads(bw):
    parser = bw.FrameParser()
    stream = bw.encode_frame(123, 4567) + bw.encode_frame(124, 4568)
    frames = []
    for i in range(len(stream)):  # one byte at a time
        frames += parser.feed(stream[i:i + 1])
    assert [f.distance_cm for f in frames] == [123, 124]
    # Odd chunk sizes, including a chunk ending on the first header byte
    parser = bw.FrameParser()
    frames = parser.feed(stream[:1]) + parser.feed(stream[1:10]) + parser.feed(stream[10:])
    assert [f.distance_cm for f in frames] == [123, 124]


def test_resynchronises_after_garbage(bw):
    parser = bw.FrameParser()
    garbage = bytes([0x00, 0x13, 0x59, 0xAA, 0x59, 0x59, 0x01, 0xFF, 0x59])
    stream = garbage + bw.encode_frame(777, 900) + b'\x10\x20' + bw.encode_frame(778, 901)
    frames = parser.feed(stream)
    assert [f.distance_cm for f in frames] == [777, 778]
    assert parser.discarded_bytes >= len(garbage)


def test_real_frame_starting_inside_a_false_header(bw):
    # A stray 0x59 right before the real header: the first candidate
    # 59 59 59 ... fails its checksum, the parser slips one byte and finds
    # the real frame.
    parser = bw.FrameParser()
    frames = parser.feed(b'\x59' + bw.encode_frame(0x5959, 0x5959))
    assert [f.distance_cm for f in frames] == [0x5959]


def test_payload_bytes_equal_to_header_are_fine(bw):
    # Distance and strength bytes of 0x59 must not be mistaken for a header.
    frame = bw.encode_frame(0x5959, 0x5959)
    parser = bw.FrameParser()
    assert [f.distance_cm for f in parser.feed(frame * 3)] == [0x5959] * 3
    assert parser.checksum_errors == 0


def test_garbage_only_does_not_grow_the_buffer(bw):
    parser = bw.FrameParser()
    for _ in range(100):
        assert parser.feed(bytes(range(0, 0x59)) * 4) == []
    assert len(parser._buffer) <= bw.FRAME_LENGTH


def test_reset_drops_a_partial_frame(bw):
    parser = bw.FrameParser()
    parser.feed(bw.encode_frame(500, 900)[:5])
    parser.reset()
    assert [f.distance_cm for f in parser.feed(bw.encode_frame(501, 900))] == [501]


# REP 117 interpretation -----------------------------------------------------

def reading(bw, model, dist, strength):
    spec = bw.MODELS[model]
    return bw.interpret(bw.Frame(dist, strength, 30.0), spec.min_range_m,
                        spec.max_range_m, spec.ambient_strength)


def test_model_limits_match_datasheets(bw):
    assert (bw.MODELS['tfmini_s'].min_range_m, bw.MODELS['tfmini_s'].max_range_m) == (0.1, 12.0)
    assert (bw.MODELS['tf_luna'].min_range_m, bw.MODELS['tf_luna'].max_range_m) == (0.2, 8.0)
    for spec in bw.MODELS.values():
        assert spec.field_of_view_rad == pytest.approx(math.radians(2.0))


@pytest.mark.parametrize('model', ['tfmini_s', 'tf_luna'])
def test_valid_reading_in_metres(bw, model):
    assert reading(bw, model, 250, 1000) == (2.5, bw.OK)
    assert reading(bw, model, 20, 100) == (0.2, bw.OK)  # strength 100 is usable


@pytest.mark.parametrize('model, dist', [
    ('tfmini_s', 0xFFFF),  # TFmini-S reports -1 when strength < 100
    ('tf_luna', 0),        # TF-Luna reports dummy_dist 0
    ('tfmini_s', 300),
])
def test_weak_signal_is_no_return(bw, model, dist):
    value, status = reading(bw, model, dist, 99)
    assert value == math.inf and status == bw.WEAK_SIGNAL


def test_saturation_is_nan(bw):
    for model in bw.MODELS:
        value, status = reading(bw, model, 300, 65535)
        assert math.isnan(value) and status == bw.SATURATED
    value, status = reading(bw, 'tfmini_s', 0xFFFE, 65535)  # -2
    assert math.isnan(value) and status == bw.SATURATED


def test_ambient_light_is_nan(bw):
    value, status = reading(bw, 'tfmini_s', 0xFFFC, 3000)  # -4
    assert math.isnan(value) and status == bw.AMBIENT_LIGHT
    # TF-Luna flags sunlight overexposure with Amp above 32768
    value, status = reading(bw, 'tf_luna', 300, 40000)
    assert math.isnan(value) and status == bw.AMBIENT_LIGHT
    assert reading(bw, 'tfmini_s', 300, 40000) == (3.0, bw.OK)


def test_out_of_range(bw):
    assert reading(bw, 'tfmini_s', 5, 2000) == (-math.inf, bw.TOO_CLOSE)
    assert reading(bw, 'tf_luna', 15, 2000) == (-math.inf, bw.TOO_CLOSE)
    assert reading(bw, 'tfmini_s', 1201, 150) == (math.inf, bw.TOO_FAR)
    assert reading(bw, 'tf_luna', 801, 150) == (math.inf, bw.TOO_FAR)
    assert reading(bw, 'tf_luna', 800, 150) == (8.0, bw.OK)


# ROS node -------------------------------------------------------------------

class FakeSerial:
    def __init__(self):
        self.pending = bytearray()
        self.fail = None
        self.closed = False

    @property
    def in_waiting(self):
        if self.fail:
            raise self.fail
        return len(self.pending)

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def close(self):
        self.closed = True


class FakeRange:
    INFRARED = 1

    def __init__(self):
        self.header = NS(stamp=None, frame_id='')


class FakeNode:
    """rclpy Node stand-in: parameters from `params`, captured publishers."""
    params = {}

    def __init__(self, name):
        self.name = name
        self.published = []
        self.publisher_args = []
        self.timers = []
        self.log = mock.MagicMock()
        self.now = 100.0

    def declare_parameter(self, name, default):
        assert name in self.params, name

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, *args):
        self.publisher_args.append(args)
        return NS(publish=self.published.append)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self.log

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: self.now))


@pytest.fixture
def node_module(monkeypatch, bw):
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['rclpy.executors'] = mock.MagicMock()
        sys.modules['sensor_msgs.msg'].Range = FakeRange
        yield load_script(SCRIPTS / 'rangefinder_node.py', 'rangefinder_node_test')


@pytest.fixture
def make_node(node_module, monkeypatch):
    clock = {'t': 100.0}
    monkeypatch.setattr(node_module.time, 'monotonic', lambda: clock['t'])

    def make(ports=None, **params):
        FakeNode.params = dict(
            port='/dev/rangefinder', baud_rate=115200, model='tfmini_s',
            topic='/aerodrone/rangefinder', frame_id='rangefinder_link',
            min_range=0.0, max_range=0.0, field_of_view=0.0,
            publish_invalid=True, stale_timeout_s=0.5,
            reconnect_period_s=1.0, poll_period_s=0.005)
        FakeNode.params.update(params)
        opened = []

        def factory(port, baud):
            item = ports.pop(0) if ports else FakeSerial()
            if isinstance(item, Exception):
                raise item
            opened.append((port, baud, item))
            return item

        node = node_module.BenewakeRangefinder(serial_factory=factory)
        node.opened, node.clock = opened, clock
        return node
    return make


def test_node_publishes_range_messages(make_node, node_module, bw):
    node = make_node()
    node._poll()  # opens the port
    port = node.opened[0][2]
    assert node.opened[0][:2] == ('/dev/rangefinder', 115200)
    port.pending += b'\x00\x59' + bw.encode_frame(250, 1000) + bw.encode_frame(251, 1000)
    node._poll()
    assert [m.range for m in node.published] == [2.5, 2.51]
    msg = node.published[0]
    assert msg.header.frame_id == 'rangefinder_link'
    assert msg.radiation_type == FakeRange.INFRARED
    assert (msg.min_range, msg.max_range) == (0.1, 12.0)
    assert msg.field_of_view == pytest.approx(math.radians(2.0))
    # Sensor-data QoS on the DemNav topic, as DemNav subscribes.
    _, topic, qos = node.publisher_args[0]
    assert topic == '/aerodrone/rangefinder'
    assert qos is node_module.qos_profile_sensor_data


def test_node_uses_model_and_overrides(make_node, bw):
    node = make_node(model='TF-Luna')
    assert (node.min_range, node.max_range) == (0.2, 8.0)
    node = make_node(model='tf_luna', min_range=0.3, max_range=6.0, field_of_view=0.05)
    assert (node.min_range, node.max_range, node.field_of_view) == (0.3, 6.0, 0.05)
    with pytest.raises(ValueError):
        make_node(model='lightware')
    with pytest.raises(ValueError):
        make_node(min_range=5.0, max_range=2.0)


def test_node_publishes_invalid_readings_per_rep117(make_node, bw):
    node = make_node()
    node._poll()
    port = node.opened[0][2]
    port.pending += (bw.encode_frame(0xFFFF, 20) + bw.encode_frame(5, 2000) +
                     bw.encode_frame(300, 65535) + bw.encode_frame(1300, 200))
    node._poll()
    values = [m.range for m in node.published]
    assert values[0] == math.inf and values[1] == -math.inf
    assert math.isnan(values[2]) and values[3] == math.inf
    # A DemNav-style consumer (finite and within limits) rejects all of them
    assert not any(math.isfinite(v) and node.min_range <= v <= node.max_range
                   for v in values)


def test_node_can_drop_invalid_readings(make_node, bw):
    node = make_node(publish_invalid=False)
    node._poll()
    node.opened[0][2].pending += bw.encode_frame(0xFFFF, 20) + bw.encode_frame(420, 900)
    node._poll()
    assert [m.range for m in node.published] == [4.2]


def test_node_retries_open_and_reconnects(make_node, bw):
    node = make_node(ports=[OSError('No such file'), FakeSerial(), FakeSerial()])
    node._poll()
    assert node._serial is None and node.log.warn.called
    node._poll()  # still inside the reconnect period
    assert node._serial is None
    node.clock['t'] += 1.0
    node._poll()
    first = node._serial
    assert first is not None
    first.pending += bw.encode_frame(100, 900)[:4]  # partial frame, then unplug
    node._poll()
    first.fail = OSError('device disconnected')
    node._poll()
    assert node._serial is None and first.closed and node.log.error.called
    node.clock['t'] += 1.0
    node._poll()
    second = node._serial
    assert second is not None and second is not first
    second.pending += bw.encode_frame(200, 900)
    node._poll()
    assert [m.range for m in node.published] == [2.0]  # stale partial frame dropped


def test_node_warns_on_stale_data(make_node, bw):
    node = make_node()
    node._poll()
    port = node.opened[0][2]
    node._check_stale()
    assert not node.log.warn.called
    node.clock['t'] += 0.6
    node._check_stale()  # opened but nothing received
    assert node.log.warn.call_count == 1
    port.pending += bw.encode_frame(200, 900)
    node._poll()
    node._check_stale()
    assert node.log.warn.call_count == 1
    assert 'resumed' in node.log.info.call_args[0][0]
    node.clock['t'] += 0.6
    node._check_stale()
    assert node.log.warn.call_count == 2
    # Garbage alone is not fresh data
    port.pending += bytes(range(40))
    node._poll()
    node.clock['t'] += 0.6
    node._check_stale()
    assert node.log.warn.call_count == 3

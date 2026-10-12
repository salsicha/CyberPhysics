"""Turret bridge driving the ST3215 servos itself (transport sts).

The bridge's Feetech STS code is a port of the turret firmware's
(applications/arduino/scripts/turret_firmware/src). Its packets are checked
against the reference bytes of the firmware's host test (captured from
Feetech's SCServo_Linux), and against the compiled firmware code on random
input. Its controller runs the firmware host test's scenarios (limits,
torque, watchdog, STOP, missing servo, fault bits, echoing adapter, and the
dart launcher's rack and trigger) against a fake serial port that answers
like the ST3215s, and the bridge's ROS side the scenarios of
test_turret_bridge.py.
"""
import math
from array import array
import random
import re
import shutil
import struct
import subprocess
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml
from conftest import REPO, install_ros_stubs, load_script

FIRMWARE = REPO / 'applications/arduino/scripts/turret_firmware'
BRIDGE_CONFIG = REPO / 'applications/pantiltros/config'
REPLY_TIMEOUT_S = 0.003  # the firmware's TURRET_SERVO_REPLY_TIMEOUT_US
PAN, TILT, RACK, TRIGGER = 12, 13, 14, 15
READ, WRITE, SYNC_WRITE = 0x02, 0x03, 0x83
STEP_RAD = 2 * math.pi / 4096


class RclpyJointState:
    """sensor_msgs/JointState as rclpy keeps it: position an array.array,
    so concatenating a list to it fails as it does in ROS."""

    def __init__(self):
        self.header = NS(stamp=None)
        self.name = []
        self._position = array('d')

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, values):
        self._position = array('d', values)


@pytest.fixture(scope='module')
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors'):
            sys.modules[name] = MagicMock()
        sys.modules['serial'] = None
        module = load_script(REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py',
                             'turret_arduino_bridge_sts')
        module.Bool = lambda data: NS(data=data)
        module.JointState = RclpyJointState
        module.KeyValue = lambda key, value: NS(key=key, value=value)
        module.DiagnosticStatus = MagicMock(OK=0, WARN=1, ERROR=2)
        module.DiagnosticArray = lambda: NS(header=NS(stamp=None), status=[])
        yield module


def hex_bytes(text):
    return bytes(int(value, 16) for value in text.split())


def checksum(body):
    return ~sum(body) & 0xFF


class Servo:
    def __init__(self, present=2048):
        self.present = present
        self.goal = present
        self.torque = False
        self.responds = True
        self.status = 0  # error byte of its status packets
        self.speed = 0
        self.acceleration = 0
        self.goal_writes = 0
        self.torque_limit = 1000  # at power-up


class FakeServoPort:
    """pyserial port to the ST3215 servos (IDs 12 and 13, and 14 and 15 for
    the launcher's rack and trigger with `launcher`) behind a half-duplex
    adapter: decodes the instruction packets written and queues the servos'
    status packets, after a copy of the packet itself with echo (one-wire
    adapters loop TX back to RX). Any packet a servo would not understand
    fails the test."""

    def __init__(self, launcher=False):
        self.port = '/dev/ttyTHS1'
        self.is_open = True
        self.echo = False
        self.servos = {PAN: Servo(), TILT: Servo()}
        if launcher:
            self.servos[RACK] = Servo(present=512)  # forward
            self.servos[TRIGGER] = Servo()  # 0: safe
        self.sent = []
        self.incoming = bytearray()
        self.fail = None  # exception raised by the next write

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, size):
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def reset_input_buffer(self):
        self.incoming.clear()

    def write(self, data):
        if self.fail is not None:
            raise self.fail
        packet = bytes(data)
        self.sent.append(packet)
        if self.echo:
            self.incoming += packet
        self._handle(packet)
        return len(packet)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def instructions(self, instruction):
        return [packet for packet in self.sent if packet[4] == instruction]

    def _reply(self, servo_id, params=b''):
        servo = self.servos[servo_id]
        if not servo.responds:
            return
        body = bytes((servo_id, len(params) + 2, servo.status)) + params
        self.incoming += b'\xff\xff' + body + bytes((checksum(body),))

    def _handle(self, p):
        assert p[:2] == b'\xff\xff' and p[3] + 4 == len(p), p.hex(' ')
        assert checksum(p[2:-1]) == p[-1], p.hex(' ')
        servo_id, instruction = p[2], p[4]
        if instruction == READ and p[5:7] == bytes((56, 2)):
            if servo_id in self.servos:
                present = self.servos[servo_id].present
                raw = (-present | 0x8000) if present < 0 else present
                self._reply(servo_id, raw.to_bytes(2, 'little'))
        elif instruction == WRITE and p[5] == 48 and len(p) == 9:  # torque limit
            servo = self.servos.get(servo_id)
            if servo is not None:
                if servo.responds:
                    servo.torque_limit = p[6] | p[7] << 8
                self._reply(servo_id)
        elif instruction == WRITE and p[5] == 40 and len(p) == 8:
            servo = self.servos.get(servo_id)
            if servo is not None:
                if servo.responds:
                    servo.torque = p[6] != 0
                self._reply(servo_id)
        elif instruction == SYNC_WRITE and servo_id == 0xFE and p[5:7] == bytes((41, 7)):
            for at in range(7, len(p) - 1, 8):
                servo = self.servos.get(p[at])
                if servo is None or not servo.responds:
                    continue
                raw = p[at + 2] | p[at + 3] << 8
                servo.acceleration = p[at + 1]
                servo.goal = -(raw & 0x7FFF) if raw & 0x8000 else raw
                assert p[at + 4:at + 6] == b'\x00\x00'  # goal time
                servo.speed = p[at + 6] | p[at + 7] << 8
                servo.goal_writes += 1
        else:
            raise AssertionError(f'unexpected packet {p.hex(" ")}')


def rack_joint(module):
    return module.StsJoint('actuator_joint', RACK, 512, 1, 0.0, 0.038, 0.0135, 550)


def trigger_joint(module):
    return module.StsJoint('trigger_joint', TRIGGER, 2048, 1, 0.0, 0.6, 0.0, 300)


def turret_config(module, actuator=False, **overrides):
    joints = (module.StsJoint('pan_joint', PAN, 2048, -1, -1.5708, 1.5708),
              module.StsJoint('tilt_joint', TILT, 2048, -1, -0.65, 0.75))
    if actuator:
        joints += (rack_joint(module), trigger_joint(module))
    return module.StsConfig(joints=joints, **overrides)


def make_rig(module, actuator=False, **overrides):
    port = FakeServoPort(launcher=actuator)
    out = NS(positions=[], errors=[])
    controller = module.StsController(
        turret_config(module, actuator, **overrides), module.StsBus(port, REPLY_TIMEOUT_S),
        lambda *position: out.positions.append(position), out.errors.append)
    return NS(port=port, servos=port.servos, out=out, controller=controller)


@pytest.fixture
def rig(module):
    return make_rig(module)


def errors_containing(rig, text):
    return sum(text in error for error in rig.out.errors)


# ---- packets: the firmware host test's reference bytes ----

# (packet the bridge builds, the reference bytes of test/host_test.cpp)
REFERENCE_PACKETS = {
    'ping 1': (lambda m: m.sts_ping(1), 'FF FF 01 02 01 FB'),
    'read 1 pos': (lambda m: m.sts_read(1, 56, 2), 'FF FF 01 04 02 38 02 BE'),
    'read 12 pos': (lambda m: m.sts_read(12, 56, 2), 'FF FF 0C 04 02 38 02 B3'),
    'torque 13 on': (lambda m: m.sts_write(13, 40, b'\x01'), 'FF FF 0D 04 03 28 01 C2'),
    'torque 12 off': (lambda m: m.sts_write(12, 40, b'\x00'), 'FF FF 0C 04 03 28 00 C4'),
    'goal 2048 speed 1000': (lambda m: m.sts_write(1, 42, bytes((0x00, 0x08, 0, 0, 0xE8, 0x03))),
                             'FF FF 01 09 03 2A 00 08 00 00 E8 03 D5'),
    'sync 12/13': (lambda m: m.sts_sync_write_position([12, 13], [1024, 3000], 2000, 50),
                   'FF FF FE 14 83 29 07 0C 32 00 04 00 00 D0 07 0D 32 B8 0B 00 00 D0 07 48'),
    'sync extremes': (lambda m: m.sts_sync_write_position([1, 2], [0, 4095], 0, 0),
                      'FF FF FE 14 83 29 07 01 00 00 00 00 00 00 00 02 00 FF 0F 00 00 00 00 29'),
    'sync negative': (lambda m: m.sts_sync_write_position([7], [-100], 500, 10),
                      'FF FF FE 0C 83 29 07 07 0A 64 80 00 00 F4 01 58'),
    # The launcher's torque limits (registers 48-49, low byte first), the
    # rack's 55 % and the trigger's 30 %: computed per the protocol, not captured
    'torque limit 14 550': (lambda m: m.sts_write(14, 48, bytes((550 & 0xFF, 550 >> 8))),
                            'FF FF 0E 05 03 30 26 02 91'),
    'torque limit 15 300': (lambda m: m.sts_write(15, 48, bytes((300 & 0xFF, 300 >> 8))),
                            'FF FF 0F 05 03 30 2C 01 8B'),
}


@pytest.mark.parametrize('name', REFERENCE_PACKETS)
def test_packets_are_the_firmware_reference_bytes(module, name):
    build, reference = REFERENCE_PACKETS[name]
    assert f'hex("{reference}")' in (FIRMWARE / 'test/host_test.cpp').read_text()
    assert build(module) == hex_bytes(reference)


def test_checksum_and_sign_magnitude_positions(module):
    assert module.sts_checksum(hex_bytes('01 02 01')) == 0xFB
    assert module.sts_encode_position(0) == 0
    assert module.sts_encode_position(4095) == 4095
    assert module.sts_encode_position(-100) == 0x8000 | 100
    assert module.sts_decode_position(0x8000 | 100) == -100
    assert module.sts_decode_position(2048) == 2048
    with pytest.raises(ValueError):
        module.sts_sync_write_position([], [], 0, 0)
    with pytest.raises(ValueError):
        module.sts_packet(1, READ, bytes(59))


def feed(parser, data):
    return [i for i, byte in enumerate(data) if parser.feed(byte)]


def test_status_parser_as_the_firmware(module):
    # Position 2048 from ID 12, with leading garbage and an extra header byte
    reply = hex_bytes('00 13 FF FF FF 0C 04 00 00 08')
    reply += bytes((checksum(reply[5:]),))
    parser = module.StsStatusParser()
    assert feed(parser, reply) == [len(reply) - 1]
    assert (parser.id, parser.error, bytes(parser.params)) == (12, 0, b'\x00\x08')
    # A bad checksum is dropped and counted
    bad = module.StsStatusParser()
    assert feed(bad, hex_bytes('FF FF 0C 04 00 00 08 00')) == []
    assert bad.bad_packets == 1
    # An echoed instruction packet is recognised as such
    read = module.sts_read(12, 56, 2)
    echo = module.StsStatusParser()
    assert feed(echo, read) == [len(read) - 1]
    assert echo.equals(read)
    assert not echo.equals(module.sts_read(13, 56, 2))


def test_conversions_and_limits_as_the_firmware(module):
    pan, tilt = turret_config(module).joints
    assert pan.to_steps(0.0) == 2048
    # pan_tilt_ros convention: steps = 2048 - rad * 4096 / 2pi
    assert pan.to_steps(math.pi / 2) == 1024
    assert pan.to_steps(-math.pi / 2) == 3072
    assert tilt.to_steps(0.75) == 2048 - 489
    assert pan.to_steps(100.0) == 0  # clamped to the servo range
    assert pan.to_steps(-100.0) == 4095
    assert pan.from_steps(1024) == pytest.approx(math.pi / 2)
    assert pan.from_steps(2048) == 0.0
    for i in range(-200, 201):
        assert pan.from_steps(pan.to_steps(i * 0.0077)) == pytest.approx(i * 0.0077,
                                                                          abs=STEP_RAD / 2)
    forward = pan._replace(direction=1, zero_steps=1000)
    assert forward.to_steps(0.1) == 1065
    assert tilt.clamp(2.0) == 0.75
    assert tilt.clamp(-2.0) == -0.65
    assert tilt.clamp(0.1) == 0.1
    # The rack: metres of travel over the pinion's pitch radius (13.5 mm)
    rack = rack_joint(module)
    assert (rack.to_steps(0.0), rack.to_steps(0.0135), rack.to_steps(0.038)) == (
        512, 512 + 652, 512 + 1835)
    assert rack.from_steps(512 + 652) == pytest.approx(0.0135, rel=STEP_RAD)
    for i in range(381):
        assert rack.from_steps(rack.to_steps(i * 1e-4)) == pytest.approx(
            i * 1e-4, abs=0.0135 * STEP_RAD / 2 + 1e-9)
    assert (rack.rest, rack.clamp(0.1), rack.clamp(-0.01)) == (0.0, 0.038, 0.0)
    trigger = trigger_joint(module)
    assert (trigger.rest, trigger.to_steps(0.6), trigger.clamp(1.0)) == (0.0, 2048 + 391, 0.6)


@pytest.mark.parametrize('joint,config', [
    ({'servo_id': TILT}, {}),  # pan and tilt share an ID
    ({'direction': 0}, {}),
    ({'lower': 0.5, 'upper': 0.5}, {}),
    ({'upper': 3.5}, {}),  # past step 0
    ({'lower': math.nan}, {}),
    ({'zero_steps': 4096}, {}),
    ({'servo_id': 254}, {}),
    ({'pinion_radius_m': -0.01}, {}),
    ({'torque_limit': 1001}, {}),
    ({}, {'watchdog_s': 0.0}),
    ({}, {'acceleration': 256}),
])
def test_unusable_config_is_rejected(module, joint, config):
    pan, tilt = turret_config(module).joints
    with pytest.raises(ValueError):
        turret_config(module, **config)._replace(joints=(pan._replace(**joint), tilt)).validate()


@pytest.mark.parametrize('rack,trigger', [
    ({'servo_id': PAN}, {}),  # shares the pan servo's ID
    ({}, {'servo_id': RACK}),  # the trigger shares the rack's
    ({'pinion_radius_m': 0.0}, {}),  # a slide needs a radius
    ({'pinion_radius_m': 0.005}, {}),  # 0.038 m is 7.6 rad: past one turn
    ({'torque_limit': 2000}, {}),
])
def test_unusable_launcher_is_rejected(module, rack, trigger):
    config = turret_config(module, actuator=True)
    config.validate()
    pan, tilt, rack_joint_, trigger_joint_ = config.joints
    with pytest.raises(ValueError):
        config._replace(joints=(pan, tilt, rack_joint_._replace(**rack),
                                trigger_joint_._replace(**trigger))).validate()


def test_settings_are_the_firmware_config(module):
    text = re.sub(r'//[^\n]*', '', (FIRMWARE / 'src/turret_config.h').read_text())
    body = text[text.index('kTurretConfig = {'):]
    joints = re.findall(r'\{"(\w+)", (\d+), (\d+), (-?\d+), (-?[\d.]+)f, (-?[\d.]+)f, '
                        r'([\d.]+)f, (\d+)\}', body)
    assert [joint[0] for joint in joints] == ['pan_joint', 'tilt_joint', 'actuator_joint',
                                              'trigger_joint']
    rest = re.search(r'\},\s*\},' + r'\s*(\w+),' * 7, body).groups()
    params = yaml.safe_load((BRIDGE_CONFIG / 'arduino_bridge.yaml').read_text())
    params = params['turret_arduino_bridge']['ros__parameters']
    for name, servo_id, zero, direction, low, high, radius, torque in joints:
        side = name.split('_')[0]
        assert (params[f'{side}_servo_id'], params[f'{side}_zero_steps'],
                params[f'{side}_direction']) == (int(servo_id), int(zero), int(direction))
        unit = 'm' if side == 'actuator' else 'rad'
        assert (params[f'{side}_min_{unit}'], params[f'{side}_max_{unit}']) == (
            float(low), float(high))
        if side == 'actuator':
            assert params['actuator_pinion_radius_m'] == float(radius) > 0
            assert params['actuator_torque_limit'] == int(torque) == 550  # priming's 0.9-1.05 N m
        elif side == 'trigger':
            assert float(radius) == 0.0
            assert params['trigger_torque_limit'] == int(torque) == 300  # stalls on the safety
        else:
            assert float(radius) == 0.0 and int(torque) == 0
    # The fitted flag is the last field; the bridge's comes from the env files
    assert re.search(r'TURRET_ACTUATOR_FITTED != 0,\s*\};', body)
    assert 'actuator_fitted' not in params
    assert (params['servo_speed'], params['servo_acceleration']) == (int(rest[0]), int(rest[1]))
    assert params['stop_disables_torque'] is (rest[6] == 'true')
    defaults = module.StsConfig._field_defaults
    assert (defaults['speed'], defaults['acceleration']) == (int(rest[0]), int(rest[1]))
    assert (defaults['watchdog_s'], defaults['feedback_period_s'], defaults['error_repeat_s'],
            defaults['torque_retry_s']) == tuple(int(ms) / 1000 for ms in rest[2:6])
    assert defaults['stop_disables_torque'] is (rest[6] == 'true')


def test_jetson_uart_env():
    lines = (BRIDGE_CONFIG / 'jetson_uart.env').read_text().splitlines()
    env = dict(line.split('=', 1) for line in lines if line and not line.startswith('#'))
    assert env == {'TURRET_ARDUINO_PORT': '/dev/ttyTHS1', 'TURRET_ARDUINO_BAUD': '1000000',
                   'TURRET_ARDUINO_TRANSPORT': 'sts', 'TURRET_ARDUINO_DRY_RUN': 'false',
                   'TURRET_ACTUATOR_FITTED': 'true'}


def test_bridge_defaults_leave_the_actuator_out(module):
    # Existing setups are unchanged: no actuator unless the env files say so
    source = (REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py').read_text()
    assert '"actuator_fitted", env_bool("TURRET_ACTUATOR_FITTED", False)' in source
    assert module.TurretArduinoBridge.actuator_fitted is False
    for name in ('arduino.env', 'arduino_uno_q.env', 'jetson_uart.env'):
        lines = (BRIDGE_CONFIG / name).read_text().splitlines()
        env = dict(line.split('=', 1) for line in lines if line and not line.startswith('#'))
        assert env['TURRET_ACTUATOR_FITTED'] == 'true', name  # both turrets carry it


# ---- the same packets and parser as the compiled firmware, on random input ----

HARNESS = r'''
// Prints what the firmware's servo code makes of each line of stdin.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "sts_servo.h"
#include "turret_core.h"

namespace sts = turret::sts;

static void hexline(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) std::printf(i ? " %02X" : "%02X", data[i]);
  std::printf("\n");
}

int main() {
  static char line[16384];
  while (std::fgets(line, sizeof(line), stdin) != NULL) {
    const char* cmd = std::strtok(line, " \n");
    std::vector<double> v;
    for (char* t = std::strtok(NULL, " \n"); t != NULL; t = std::strtok(NULL, " \n")) {
      v.push_back(std::strtod(t, NULL));
    }
    uint8_t buf[sts::kMaxPacket];
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < v.size(); ++i) bytes.push_back(static_cast<uint8_t>(v[i]));
    if (std::strcmp(cmd, "sync") == 0) {  // speed acc id steps [id steps ...]
      std::vector<uint8_t> ids;
      std::vector<int32_t> steps;
      for (size_t i = 2; i + 1 < v.size(); i += 2) {
        ids.push_back(static_cast<uint8_t>(v[i]));
        steps.push_back(static_cast<int32_t>(v[i + 1]));
      }
      hexline(buf, sts::buildSyncWritePosition(buf, sizeof(buf), &ids[0], &steps[0], ids.size(),
                                               static_cast<uint16_t>(v[0]), bytes[1]));
    } else if (std::strcmp(cmd, "read") == 0) {  // id address count
      hexline(buf, sts::buildRead(buf, sizeof(buf), bytes[0], bytes[1], bytes[2]));
    } else if (std::strcmp(cmd, "write") == 0) {  // id address data...
      hexline(buf, sts::buildWrite(buf, sizeof(buf), bytes[0], bytes[1], &bytes[0] + 2,
                                   bytes.size() - 2));
    } else if (std::strcmp(cmd, "status") == 0) {  // received bytes
      sts::StatusParser parser;
      for (size_t i = 0; i < bytes.size(); ++i) {
        if (parser.feed(bytes[i])) {
          std::printf("%u %u", parser.id(), parser.error());
          for (size_t j = 0; j < parser.numParams(); ++j) std::printf(" %u", parser.params()[j]);
          std::printf(";");
        }
      }
      std::printf("bad %u\n", static_cast<unsigned>(parser.badPackets()));
    } else {  // steps|position zero direction pinion_radius value
      const turret::JointConfig joint = {"j", 1, static_cast<int32_t>(v[0]),
                                         static_cast<int8_t>(v[1]), -10.0f, 10.0f,
                                         static_cast<float>(v[2]), 0};
      if (std::strcmp(cmd, "steps") == 0) {
        std::printf("%d\n", static_cast<int>(turret::toSteps(joint, static_cast<float>(v[3]))));
      } else {
        std::printf("%.9g\n", turret::fromSteps(joint, static_cast<int32_t>(v[3])));
      }
    }
  }
  return 0;
}
'''


@pytest.fixture(scope='module')
def firmware_harness(tmp_path_factory):
    compiler = shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
    if compiler is None:
        pytest.skip('no C++ compiler')
    directory = tmp_path_factory.mktemp('sts_harness')
    (directory / 'harness.cpp').write_text(HARNESS)
    out = directory / 'harness'
    sources = [str(directory / 'harness.cpp')]
    sources += sorted(str(path) for path in (FIRMWARE / 'src').glob('*.cpp'))
    result = subprocess.run([compiler, '-std=c++11', '-O1', f'-I{FIRMWARE / "src"}', *sources,
                             '-o', str(out)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    return out


def f32(value):
    """The float32 nearest value, as the firmware stores it."""
    return struct.unpack('f', struct.pack('f', value))[0]


def random_status_stream(rng):
    data = bytearray()
    for _ in range(rng.randint(1, 6)):
        kind = rng.random()
        if kind < 0.5:  # a status packet, sometimes with a wrong checksum
            params = bytes(rng.randrange(256) for _ in range(rng.randint(0, 8)))
            body = bytes((rng.randrange(254), len(params) + 2, rng.randrange(256))) + params
            sum_byte = checksum(body) ^ (rng.randrange(1, 256) if rng.random() < 0.2 else 0)
            data += b'\xff' * rng.randint(2, 3) + body + bytes((sum_byte,))
        elif kind < 0.7:  # a header with an impossible length
            data += bytes((0xFF, 0xFF, rng.randrange(254), rng.choice((0, 1, 61, 200, 255))))
        else:
            data += bytes(rng.choice((0xFF, rng.randrange(256))) for _ in range(rng.randint(1, 6)))
    return bytes(data)


def test_matches_the_compiled_firmware_on_random_input(module, firmware_harness):
    rng = random.Random(3215)
    cases = []  # (stdin line, what the bridge makes of it)
    for _ in range(300):
        count = rng.randint(1, 8)
        ids = [rng.randrange(254) for _ in range(count)]
        steps = [rng.randint(-40000, 40000) for _ in range(count)]
        speed, acceleration = rng.randrange(65536), rng.randrange(256)
        try:
            packet = module.sts_sync_write_position(ids, steps, speed, acceleration).hex(' ')
        except ValueError:
            packet = ''  # too long for a 64-byte packet: the firmware builds nothing either
        pairs = ' '.join(f'{i} {s}' for i, s in zip(ids, steps))
        cases.append((f'sync {speed} {acceleration} {pairs}', packet.upper()))
        servo_id, address = rng.randrange(254), rng.randrange(256)
        count = rng.randrange(256)
        cases.append((f'read {servo_id} {address} {count}',
                      module.sts_read(servo_id, address, count).hex(' ').upper()))
        data = bytes(rng.randrange(256) for _ in range(rng.randint(0, 8)))
        cases.append((f'write {servo_id} {address} {" ".join(map(str, data))}'.strip(),
                      module.sts_write(servo_id, address, data).hex(' ').upper()))
        stream = random_status_stream(rng)
        parser = module.StsStatusParser()
        parsed = ''
        for byte in stream:
            if parser.feed(byte):
                parsed += ' '.join(map(str, (parser.id, parser.error, *parser.params))) + ';'
        cases.append((f'status {" ".join(map(str, stream))}', f'{parsed}bad {parser.bad_packets}'))
    result = subprocess.run([str(firmware_harness)], input=''.join(f'{c}\n' for c, _ in cases),
                            capture_output=True, text=True, timeout=60, check=True)
    lines = result.stdout.splitlines()
    assert len(lines) == len(cases)
    for (command, expected), line in zip(cases, lines):
        assert line == expected, command

    # Joint positions <-> steps, revolute (radians) and slides (metres over a
    # pinion radius): the firmware rounds in float32, so the two may differ by
    # one step only where the exact value is within float error of a half step
    conversions = []
    for _ in range(2000):
        zero, direction = rng.randrange(4096), rng.choice((1, -1))
        radius = rng.choice((0.0, f32(rng.uniform(0.004, 0.02))))
        value = rng.uniform(-4.0, 4.0) * (radius or 1.0)
        conversions.append((zero, direction, radius, f32(value), rng.randrange(4096)))
    script = ''.join(f'steps {z} {d} {r!r} {v!r}\nposition {z} {d} {r!r} {s}\n'
                     for z, d, r, v, s in conversions)
    result = subprocess.run([str(firmware_harness)], input=script, capture_output=True,
                            text=True, timeout=60, check=True)
    values = result.stdout.splitlines()
    for (zero, direction, radius, value, steps), fw_steps, fw_value in zip(
            conversions, values[::2], values[1::2]):
        joint = module.StsJoint('j', 1, zero, direction, -10.0, 10.0, radius)
        exact = zero + direction * joint.shaft_angle(value) * module.STS_STEPS_PER_RAD
        near_half = abs(exact - math.floor(exact) - 0.5) < 0.01
        assert abs(joint.to_steps(value) - int(fw_steps)) <= (1 if near_half else 0), (
            zero, radius, value)
        assert joint.from_steps(steps) == pytest.approx(float(fw_value), abs=2e-6 * (radius or 1))


# ---- controller: the firmware host test's scenarios ----

def test_idle_reads_positions_and_writes_nothing(module, rig):
    rig.servos[PAN].present = 1024  # pan at +pi/2
    rig.controller.update(0.0)
    assert rig.out.positions == [pytest.approx((math.pi / 2, 0.0), abs=1e-9)]
    rig.controller.update(0.02)  # within the 40 ms feedback period
    assert len(rig.out.positions) == 1
    rig.controller.update(0.04)
    assert len(rig.out.positions) == 2
    assert {p[4] for p in rig.port.sent} == {READ}
    assert rig.controller.state == module.StsController.IDLE
    assert rig.controller.have_feedback
    assert not rig.servos[PAN].torque


def test_first_set_enables_torque_then_only_goals_are_clamped_and_written(module, rig):
    assert rig.controller.set(0.5, -0.25, 0.05)
    assert rig.controller.state == module.StsController.ACTIVE
    assert rig.servos[PAN].torque and rig.servos[TILT].torque
    assert len(rig.port.instructions(WRITE)) == 2
    assert (rig.servos[PAN].goal, rig.servos[TILT].goal) == (1722, 2211)
    assert (rig.servos[PAN].speed, rig.servos[PAN].acceleration) == (2000, 50)
    assert len(rig.port.instructions(SYNC_WRITE)) == 1  # both servos in one packet
    # Later SETs only write goals; out-of-range goals are clamped to the limits
    assert rig.controller.set(3.0, 2.0, 0.06)
    assert len(rig.port.instructions(WRITE)) == 2
    assert (rig.servos[PAN].goal, rig.servos[TILT].goal) == (1024, 1559)
    assert rig.controller.goal == [1.5708, 0.75]
    assert rig.controller.set(-3.0, -2.0, 0.07)
    assert (rig.servos[PAN].goal, rig.servos[TILT].goal) == (3072, 2472)
    # Non-finite values are rejected with an error and move nothing
    writes = rig.servos[PAN].goal_writes
    assert not rig.controller.set(math.nan, 0.0, 0.08)
    assert not rig.controller.set(0.0, math.inf, 0.08)
    assert rig.servos[PAN].goal_writes == writes
    assert errors_containing(rig, 'non-finite') == 2


def test_watchdog_holds_the_measured_position(module, rig):
    rig.controller.set(0.2, 0.1, 1.0)
    rig.servos[PAN].present, rig.servos[TILT].present = 1900, 2000  # still on their way
    writes = rig.servos[PAN].goal_writes
    rig.controller.update(1.499)
    assert rig.controller.state == module.StsController.ACTIVE
    assert rig.servos[PAN].goal_writes == writes
    # 500 ms without SET: hold where the servos are, torque on, no error
    rig.controller.update(1.5)
    assert rig.controller.state == module.StsController.HOLDING
    assert (rig.servos[PAN].goal, rig.servos[TILT].goal) == (1900, 2000)
    assert rig.servos[PAN].torque
    assert rig.out.errors == []
    held = rig.servos[PAN].goal_writes
    for k in range(1, 150):
        rig.controller.update(1.5 + k * 0.01)
    assert rig.servos[PAN].goal_writes == held
    # A SET resumes (re-enabling torque), STOP holds again
    torque_writes = len(rig.port.instructions(WRITE))
    assert rig.controller.set(-0.3, 0.0, 3.0)
    assert rig.controller.state == module.StsController.ACTIVE
    assert len(rig.port.instructions(WRITE)) == torque_writes + 2
    rig.servos[PAN].present = 2100
    rig.controller.stop(3.01)
    assert rig.controller.state == module.StsController.HOLDING
    assert rig.servos[PAN].goal == 2100


def test_stop_before_any_set_holds_the_present_position(module, rig):
    rig.servos[PAN].present = 1500
    rig.controller.stop(0.005)
    assert rig.servos[PAN].goal == 1500
    assert rig.controller.state == module.StsController.HOLDING
    assert rig.port.instructions(WRITE) == []  # torque untouched


def test_stop_can_disable_torque_instead(module):
    rig = make_rig(module, stop_disables_torque=True)
    rig.controller.set(0.0, 0.0, 0.0)
    assert rig.servos[PAN].torque
    rig.controller.stop(0.01)
    assert not rig.servos[PAN].torque and not rig.servos[TILT].torque
    assert rig.controller.set(0.1, 0.0, 0.02)
    assert rig.servos[PAN].torque and rig.servos[TILT].torque


def test_missing_servo_is_reported_at_most_once_per_second(module, rig):
    rig.servos[TILT].responds = False
    rig.controller.update(0.04)
    assert rig.out.positions == []  # no position while a joint is not measured
    assert rig.out.errors == ['servo 13 (tilt_joint) timeout']
    assert not rig.controller.have_feedback
    for k in range(2, 26):  # up to 1.0 s
        rig.controller.update(k * 0.04 + 0.001)
    assert errors_containing(rig, 'timeout') == 1
    rig.controller.update(1.05)
    assert errors_containing(rig, 'timeout') == 2
    # Recovery: positions again, and the next failure is reported at once
    rig.servos[TILT].responds = True
    rig.controller.update(1.1)
    assert len(rig.out.positions) == 1
    rig.servos[TILT].responds = False
    rig.controller.update(1.15)
    assert errors_containing(rig, 'timeout') == 3
    # A torque enable failure is reported; a SET still writes goals
    rig.controller.set(0.1, 0.1, 1.16)
    assert errors_containing(rig, 'torque enable timeout') <= 1
    assert rig.servos[PAN].torque
    assert rig.servos[PAN].goal == 1983
    # While the servo stays silent, torque re-enable is retried at most every 250 ms
    writes = len(rig.port.instructions(WRITE))
    rig.controller.set(0.1, 0.1, 1.17)
    assert len(rig.port.instructions(WRITE)) == writes
    rig.controller.set(0.1, 0.1, 1.42)
    assert len(rig.port.instructions(WRITE)) == writes + 2
    # STOP cannot hold a joint that does not answer, and says so
    rig.controller.stop(1.5)
    assert errors_containing(rig, 'servo 13 (tilt_joint) cannot hold: no position reply') == 1


def test_fault_bits_are_reported_and_the_position_is_still_valid(module, rig):
    rig.servos[PAN].status = 0x20
    rig.controller.update(0.04)
    assert len(rig.out.positions) == 1
    assert rig.out.errors == ['servo 12 (pan_joint) fault 0x20 overload']
    rig.controller.update(0.08)  # persisting: not repeated within the second
    assert len(rig.out.errors) == 1
    rig.servos[TILT].status = 0x0D
    rig.controller.update(0.12)
    assert rig.out.errors[-1] == 'servo 13 (tilt_joint) fault 0x0D voltage overheat overcurrent'


def test_echoing_adapter(module, rig):
    rig.port.echo = True
    rig.servos[PAN].present = 3000
    rig.controller.update(0.04)
    assert rig.out.errors == []
    assert rig.out.positions == [pytest.approx((-(3000 - 2048) * STEP_RAD, 0.0))]
    assert rig.controller.set(0.0, 0.0, 0.05)
    assert rig.out.errors == []
    assert rig.servos[PAN].torque
    rig.servos[TILT].responds = False
    rig.controller.update(0.09)
    assert rig.out.errors == ['servo 13 (tilt_joint) timeout']
    assert rig.controller.bus.bad_packets == 0


def test_garbled_reply_is_a_bad_reply(module, rig):
    rig.controller.update(0.0)
    original = rig.port._reply

    def corrupt(servo_id, params=b''):
        original(servo_id, params)
        if servo_id == TILT:
            rig.port.incoming[-1] ^= 0xFF
    rig.port._reply = corrupt
    rig.controller.update(0.04)
    assert rig.out.errors == ['servo 13 (tilt_joint) bad reply']
    assert rig.controller.bus.bad_packets == 1


# ---- the bridge's ROS side over the sts transport ----

@pytest.fixture
def port():
    return FakeServoPort()


@pytest.fixture
def bridge(module, port):
    node = module.TurretArduinoBridge.__new__(module.TurretArduinoBridge)
    node.get_logger = MagicMock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp'))
    node.transport = 'sts'
    node.serial_port = '/dev/ttyTHS1'
    node.baud_rate = 1000000
    node.dry_run = False
    node.pan_min_rad, node.pan_max_rad = -1.5708, 1.5708
    node.tilt_min_rad, node.tilt_max_rad = -0.65, 0.75
    node.sts_config = turret_config(module)
    node.servo_reply_timeout_s = REPLY_TIMEOUT_S
    node.servos = None
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
    node.opened = []

    def open_port(path, baud, **options):
        node.opened.append((path, baud, options))
        port.is_open = True
        return port
    module.serial = NS(Serial=open_port)
    yield node
    module.serial = None


def command(pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt])


def last_ready(bridge):
    return bridge.ready_pub.publish.call_args[0][0].data


def published_positions(bridge):
    return [list(call[0][0].position) for call in bridge.joint_state_pub.publish.call_args_list]


def test_connects_at_1_mbps_and_is_ready_at_once(bridge, port):
    bridge._poll()
    assert bridge.opened == [('/dev/ttyTHS1', 1000000,
                              {'timeout': 0.001, 'write_timeout': 0.1, 'exclusive': True})]
    assert bridge.actuator_ready()  # no Arduino reset to wait for
    assert last_ready(bridge) is True
    assert published_positions(bridge) == [[0.0, 0.0]]  # measured, both servos at 2048
    assert {p[4] for p in port.sent} == {READ}
    assert bridge.last_error == ''


def test_nothing_is_written_before_the_first_set_or_stop(bridge, port, module):
    for k in range(100):
        with patch.object(module.time, 'monotonic', return_value=100.0 + k * 0.02):
            bridge._poll()
    assert {p[4] for p in port.sent} == {READ}
    assert len(published_positions(bridge)) == 50  # 25 Hz from a 50 Hz poll
    assert not port.servos[PAN].torque
    assert bridge.servos.state == 'idle'


def test_command_is_clamped_written_and_never_echoed(bridge, port):
    bridge._poll()
    bridge.joint_state_pub.publish.reset_mock()
    bridge._joint_command(command(0.5, 2.0))
    assert port.servos[PAN].torque and port.servos[TILT].torque
    assert (port.servos[PAN].goal, port.servos[TILT].goal) == (1722, 1559)  # tilt at 0.75
    assert bridge.last_write_time is not None
    bridge.joint_state_pub.publish.assert_not_called()


def test_non_finite_command_is_rejected(bridge, port):
    bridge._poll()
    for value in (math.nan, math.inf):
        bridge._joint_command(command(value, 0.0))
        bridge._joint_command(command(0.0, value))
    assert {p[4] for p in port.sent} == {READ}
    assert 'non-finite' in bridge.last_error


def test_emergency_stop_holds_and_blocks_commands(bridge, port):
    bridge._poll()
    bridge._joint_command(command(0.2, 0.1))
    port.servos[PAN].present = 1950
    bridge._emergency_stop(NS(data=True))
    assert bridge.servos.state == 'holding'
    assert port.servos[PAN].goal == 1950  # held where it is
    assert last_ready(bridge) is False
    writes = len(port.instructions(SYNC_WRITE))
    bridge._joint_command(command(0.1, 0.1))
    bridge._poll()  # STOP is not repeated
    assert len(port.instructions(SYNC_WRITE)) == writes
    assert bridge.dropped_commands == 1
    bridge._emergency_stop(NS(data=False))
    assert last_ready(bridge) is True
    torque_writes = len(port.instructions(WRITE))
    bridge._joint_command(command(0.1, 0.1))
    assert len(port.instructions(WRITE)) == torque_writes + 2  # torque re-enabled
    assert port.servos[PAN].goal == 1983


def test_stop_is_sent_once_connected_when_release_required(bridge, port):
    bridge.estop_engaged = True  # require_estop_release default
    port.servos[PAN].present = 2222
    bridge._poll()
    assert port.servos[PAN].goal == 2222
    assert len(port.instructions(SYNC_WRITE)) == 1
    assert port.instructions(WRITE) == []  # STOP never enables torque
    bridge._poll()
    assert len(port.instructions(SYNC_WRITE)) == 1
    assert not bridge.actuator_ready()


def test_watchdog_holds_when_commands_stop(bridge, port, module):
    with patch.object(module.time, 'monotonic', return_value=100.0):
        bridge._poll()
        bridge._joint_command(command(0.3, 0.0))
    port.servos[PAN].present = 1900
    with patch.object(module.time, 'monotonic', return_value=100.49):
        bridge._poll()
    assert bridge.servos.state == 'active'
    with patch.object(module.time, 'monotonic', return_value=100.5):
        bridge._poll()
    assert bridge.servos.state == 'holding'
    assert port.servos[PAN].goal == 1900
    assert bridge.actuator_ready()  # a new command resumes
    assert bridge.last_error == ''


def test_missing_servo_gives_errors_and_no_joint_states(bridge, port, module):
    module.DiagnosticStatus.ERROR = 2
    port.servos[TILT].responds = False
    bridge._poll()
    bridge.joint_state_pub.publish.assert_not_called()
    assert bridge.last_error == 'servo 13 (tilt_joint) timeout'
    bridge._publish_diagnostics()
    status = bridge.diagnostics_pub.publish.call_args[0][0].status[0]
    assert status.level == 2
    assert status.message == 'servo 13 (tilt_joint) timeout'
    # As with the firmware transports, the link is up: commands still go out
    assert bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.0))
    assert port.servos[PAN].goal == 1983


def test_fault_bit_is_an_error_but_positions_flow(bridge, port):
    port.servos[PAN].status = 0x04
    port.servos[PAN].present = 1024
    bridge._poll()
    assert published_positions(bridge) == [[pytest.approx(math.pi / 2), 0.0]]
    assert bridge.last_error == 'servo 12 (pan_joint) fault 0x04 overheat'


def test_echoing_adapter_through_the_bridge(bridge, port):
    port.echo = True
    port.servos[TILT].present = 1800
    bridge._poll()
    bridge._joint_command(command(-0.4, 0.2))
    assert published_positions(bridge) == [[0.0, pytest.approx((2048 - 1800) * STEP_RAD)]]
    assert (port.servos[PAN].goal, port.servos[TILT].goal) == (2309, 1918)
    assert bridge.last_error == ''
    assert bridge.servos.bus.bad_packets == 0


def test_port_failure_closes_and_reconnects_idle(bridge, port):
    bridge._poll()
    bridge._joint_command(command(0.1, 0.1))
    port.fail = OSError('write failed: [Errno 5] Input/output error')
    bridge._joint_command(command(0.2, 0.1))
    assert not bridge._connected()
    assert not bridge.actuator_ready()
    assert not port.is_open
    assert bridge.last_error.startswith('servo bus: write failed')
    bridge._joint_command(command(0.3, 0.1))
    assert bridge.dropped_commands == 1
    bridge._poll()  # within reconnect_interval_s: no new attempt
    assert len(bridge.opened) == 1
    port.fail = None
    bridge.last_connect_attempt = float('-inf')
    bridge._poll()
    assert bridge._connected() and bridge.actuator_ready()
    assert bridge.servos.state == 'idle'  # a fresh controller: torque again at the next SET
    writes = len(port.instructions(WRITE))
    bridge._joint_command(command(0.2, 0.1))
    assert len(port.instructions(WRITE)) == writes + 2


def test_stop_on_shutdown(bridge, port):
    bridge._poll()
    bridge._joint_command(command(0.1, 0.1))
    port.servos[PAN].present = 2000
    assert bridge._send_stop()
    bridge._close_serial()
    assert port.servos[PAN].goal == 2000
    assert not port.is_open
    assert bridge.servos is None


def test_dry_run_is_never_ready(bridge, port):
    bridge.dry_run = True
    bridge._poll()
    assert bridge.opened == []
    assert not bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.dropped_commands == 1


def test_diagnostics_name_the_servo_bus(bridge, port, module):
    module.DiagnosticStatus.OK = 0
    bridge._poll()
    bridge._publish_diagnostics()
    status = bridge.diagnostics_pub.publish.call_args[0][0].status[0]
    values = {kv.key: kv.value for kv in status.values}
    assert status.level == 0
    assert status.message == 'servo bus connected'
    assert status.hardware_id == '/dev/ttyTHS1'
    assert values['transport'] == 'sts'
    assert values['port'] == '/dev/ttyTHS1'
    assert values['baud_rate'] == '1000000'
    assert values['servo_ids'] == '12,13'
    assert values['servo_state'] == 'idle'
    assert values['ready'] == 'true'
    assert values['position_feedback'] == 'true'
    bridge.last_feedback_time -= 2.0
    bridge._publish_diagnostics()
    status = bridge.diagnostics_pub.publish.call_args[0][0].status[0]
    assert status.message == 'servo bus connected, no position feedback from servos'


def test_servo_settings_come_from_the_parameters(bridge, module):
    params = yaml.safe_load((BRIDGE_CONFIG / 'arduino_bridge.yaml').read_text())
    params = params['turret_arduino_bridge']['ros__parameters']
    bridge.get_parameter = lambda name: NS(value=params[name])
    assert bridge._sts_config() == turret_config(module)
    params['tilt_servo_id'] = 12
    with pytest.raises(ValueError, match='servo ids must differ'):
        bridge._sts_config()


# ---- the dart launcher (rack ID 14, trigger ID 15) ----

def rack_steps(module, metres):
    return rack_joint(module).to_steps(metres)


def trigger_steps(module, rad):
    return trigger_joint(module).to_steps(rad)


def test_launcher_set_and_torque_limit(module):
    rig = make_rig(module, actuator=True)
    rig.servos[RACK].present = rack_steps(module, 0.002)
    rig.controller.update(0.0)
    assert rig.out.positions == [pytest.approx((0.0, 0.0, 0.002, 0.0), abs=1e-4)]
    assert {p[4] for p in rig.port.sent} == {READ}  # idle: nothing written
    # A SET naming both: torque on for all four and the launcher's limits, one sync write
    assert rig.controller.set(0.1, 0.0, 0.05, actuator=0.038, trigger=0.0)
    assert len(rig.port.instructions(WRITE)) == 6
    assert rig.servos[RACK].torque and rig.servos[TRIGGER].torque
    assert (rig.servos[RACK].torque_limit, rig.servos[TRIGGER].torque_limit,
            rig.servos[PAN].torque_limit) == (550, 300, 1000)
    assert len(rig.port.instructions(SYNC_WRITE)) == 1
    assert (rig.servos[RACK].goal, rig.servos[TRIGGER].goal) == (2347, 2048)
    # Clamped to the limits
    rig.controller.set(0.1, 0.0, 0.06, actuator=0.5, trigger=2.0)
    assert (rig.servos[RACK].goal, rig.servos[TRIGGER].goal) == (2347, trigger_steps(module, 0.6))
    rig.controller.set(0.1, 0.0, 0.07, actuator=-0.5, trigger=-1.0)
    assert (rig.servos[RACK].goal, rig.servos[TRIGGER].goal) == (512, 2048)
    # Without the trigger it returns to 0; without the rack, the rack stops
    # where it is (its present position)
    rig.controller.set(0.1, 0.0, 0.08, actuator=0.02, trigger=0.6)
    rig.servos[RACK].present = rack_steps(module, 0.017)  # on its way back
    rig.controller.set(0.1, 0.0, 0.09)
    assert rig.servos[TRIGGER].goal == 2048 and rig.controller.goal[3] == 0.0
    assert rig.servos[RACK].goal == rack_steps(module, 0.017)
    rig.controller.set(0.1, 0.0, 0.095, trigger=0.3)
    assert rig.servos[TRIGGER].goal == trigger_steps(module, 0.3)
    assert rig.servos[RACK].goal == rack_steps(module, 0.017)
    writes = rig.servos[RACK].goal_writes
    assert not rig.controller.set(0.1, 0.0, 0.1, actuator=math.nan, trigger=0.0)
    assert not rig.controller.set(0.1, 0.0, 0.1, actuator=0.0, trigger=math.inf)
    assert rig.servos[RACK].goal_writes == writes


def test_launcher_watchdog_and_stop_hold_the_rack_and_return_the_trigger(module):
    rig = make_rig(module, actuator=True)
    rig.controller.set(0.2, 0.1, 1.0, actuator=0.038, trigger=0.6)
    rig.servos[PAN].present, rig.servos[TILT].present = 1900, 2000
    rig.servos[RACK].present = rack_steps(module, 0.017)
    rig.servos[TRIGGER].present = trigger_steps(module, 0.3)
    rig.controller.update(1.499)
    assert rig.servos[RACK].goal == 2347 and rig.servos[TRIGGER].goal == trigger_steps(module, 0.6)
    rig.controller.update(1.5)
    assert rig.controller.state == module.StsController.HOLDING
    assert (rig.servos[PAN].goal, rig.servos[TILT].goal) == (1900, 2000)
    assert rig.servos[RACK].goal == rack_steps(module, 0.017)  # stopped where it was
    assert rig.servos[TRIGGER].goal == 2048
    assert rig.out.errors == []
    rig.controller.set(0.2, 0.1, 2.0, actuator=0.038, trigger=0.6)
    rig.servos[RACK].present = rack_steps(module, 0.025)
    rig.controller.stop(2.01)
    assert (rig.servos[RACK].goal, rig.servos[TRIGGER].goal) == (rack_steps(module, 0.025), 2048)
    # STOP before any SET: trigger to 0, rack held, torque untouched
    idle = make_rig(module, actuator=True)
    idle.servos[RACK].present, idle.servos[TRIGGER].present = 3000, 2300
    idle.controller.stop(0.005)
    assert (idle.servos[RACK].goal, idle.servos[TRIGGER].goal) == (3000, 2048)
    assert idle.port.instructions(WRITE) == []
    # stop_disables_torque applies to pan and tilt; the launcher stays safe and powered
    limp = make_rig(module, actuator=True, stop_disables_torque=True)
    limp.controller.set(0.0, 0.0, 0.0, actuator=0.038, trigger=0.6)
    limp.servos[RACK].present = rack_steps(module, 0.02)
    limp.controller.stop(0.01)
    assert not limp.servos[PAN].torque and not limp.servos[TILT].torque
    assert limp.servos[RACK].torque and limp.servos[TRIGGER].torque
    assert (limp.servos[RACK].goal, limp.servos[TRIGGER].goal) == (rack_steps(module, 0.02), 2048)
    # A silent rack cannot be held, and says so; the trigger still returns to 0
    silent = make_rig(module, actuator=True)
    silent.controller.set(0.0, 0.0, 0.0, actuator=0.038, trigger=0.6)
    silent.servos[RACK].responds = False
    silent.controller.stop(0.01)
    assert silent.servos[TRIGGER].goal == 2048
    assert errors_containing(silent, 'servo 14 (actuator_joint) cannot hold') == 1


def test_missing_launcher_servo_is_reported_and_pan_tilt_still_flow(module):
    rig = make_rig(module, actuator=True)
    rig.servos[TRIGGER].responds = False
    rig.controller.update(0.04)
    assert rig.out.positions == [(0.0, 0.0)]  # pan and tilt, without the launcher
    assert rig.out.errors == ['servo 15 (trigger_joint) timeout']
    assert rig.controller.have_feedback and not rig.controller.have_launcher
    for k in range(2, 26):
        rig.controller.update(k * 0.04 + 0.001)
    assert errors_containing(rig, 'servo 15 (trigger_joint) timeout') == 1
    rig.controller.update(1.05)
    assert errors_containing(rig, 'servo 15 (trigger_joint) timeout') == 2
    rig.servos[TRIGGER].responds = True
    rig.controller.update(1.1)
    assert len(rig.out.positions[-1]) == 4
    rig.servos[RACK].status = 0x20
    rig.controller.update(1.15)
    assert rig.out.errors[-1] == 'servo 14 (actuator_joint) fault 0x20 overload'
    assert len(rig.out.positions[-1]) == 4


def test_without_a_launcher_fitted_its_goals_are_ignored(module):
    rig = make_rig(module)
    rig.port.servos[RACK], rig.port.servos[TRIGGER] = Servo(present=512), Servo()
    assert rig.controller.set(0.3, 0.0, 0.0, actuator=0.038, trigger=0.6)
    assert rig.servos[PAN].goal == pan_steps(module, 0.3)
    for k in range(1, 50):
        rig.controller.set(0.3, 0.0, k * 0.02, actuator=0.038, trigger=0.6)
    assert errors_containing(rig, 'ignored: no launcher fitted') == 1
    rig.controller.set(0.3, 0.0, 1.0, actuator=0.038)
    assert errors_containing(rig, 'ignored: no launcher fitted') == 2
    rig.controller.update(1.04)
    rig.controller.stop(1.05)
    assert len(rig.out.positions[-1]) == 2
    assert rig.servos[RACK].goal_writes == 0 and rig.servos[TRIGGER].goal_writes == 0
    assert not rig.servos[RACK].torque and not rig.servos[TRIGGER].torque


def pan_steps(module, rad):
    return turret_config(module).joints[0].to_steps(rad)


@pytest.fixture
def launcher_port():
    return FakeServoPort(launcher=True)


@pytest.fixture
def launcher_bridge(module, bridge, launcher_port):
    bridge.actuator_fitted = True
    bridge.sts_config = turret_config(module, actuator=True)
    module.serial = NS(Serial=lambda path, baud, **options: (
        bridge.opened.append((path, baud, options)), launcher_port)[1])
    return bridge


def command4(pan, tilt, actuator=None, trigger=None):
    names, values = ['pan_joint', 'tilt_joint'], [pan, tilt]
    for name, value in (('actuator_joint', actuator), ('trigger_joint', trigger)):
        if value is not None:
            names.append(name)
            values.append(value)
    return NS(name=names, position=values)


def published_states(bridge):
    return [(list(call[0][0].name), list(call[0][0].position))
            for call in bridge.joint_state_pub.publish.call_args_list]


def test_bridge_publishes_and_commands_the_launcher(module, launcher_bridge, launcher_port):
    launcher_port.servos[RACK].present = rack_steps(module, 0.001)
    launcher_bridge._poll()
    [(names, position)] = published_states(launcher_bridge)
    assert names == ['pan_joint', 'tilt_joint', 'actuator_joint', 'trigger_joint']
    assert position == pytest.approx([0.0, 0.0, 0.001, 0.0], abs=1e-4)
    assert launcher_port.servos[RACK].torque_limit == 1000  # nothing written yet
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.038, 0.0))
    assert launcher_port.servos[RACK].goal == 2347 and launcher_port.servos[TRIGGER].goal == 2048
    assert launcher_port.servos[RACK].torque_limit == 550
    assert launcher_port.servos[TRIGGER].torque_limit == 300
    launcher_bridge._joint_command(command4(0.1, 0.0, 9.0, 9.0))  # clamped
    assert launcher_port.servos[RACK].goal == 2347
    assert launcher_port.servos[TRIGGER].goal == trigger_steps(module, 0.6)
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.02))  # no trigger_joint: 0
    assert launcher_port.servos[TRIGGER].goal == 2048
    launcher_port.servos[RACK].present = rack_steps(module, 0.011)
    launcher_bridge._joint_command(command4(0.1, 0.0))  # no launcher: trigger 0, rack stopped
    assert launcher_port.servos[RACK].goal == rack_steps(module, 0.011)
    launcher_bridge._joint_command(command4(0.1, 0.0, trigger=0.6))  # trigger without the rack
    assert launcher_port.servos[RACK].goal == rack_steps(module, 0.011)
    assert launcher_port.servos[TRIGGER].goal == 2048
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.02, math.nan))  # rejected whole
    assert 'non-finite' in launcher_bridge.last_error
    # The emergency stop returns the trigger to 0, stops the rack, blocks commands
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.038, 0.6))
    launcher_port.servos[RACK].present = rack_steps(module, 0.033)
    launcher_bridge._emergency_stop(NS(data=True))
    assert launcher_port.servos[RACK].goal == rack_steps(module, 0.033)
    assert launcher_port.servos[TRIGGER].goal == 2048
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.038, 0.6))
    assert launcher_port.servos[TRIGGER].goal == 2048
    module.DiagnosticStatus.OK = 0
    launcher_bridge.servos.next_feedback_time = float('-inf')  # read now
    launcher_bridge._poll()
    launcher_bridge._publish_diagnostics()
    values = {kv.key: kv.value
              for kv in launcher_bridge.diagnostics_pub.publish.call_args[0][0].status[0].values}
    assert values['servo_ids'] == '12,13,14,15' and values['actuator_fitted'] == 'true'
    assert values['actuator_position_m'] == '0.0330' and values['trigger_position_rad'] == '0.000'


def test_bridge_watchdog_and_shutdown_make_the_launcher_safe(module, launcher_bridge,
                                                            launcher_port):
    with patch.object(module.time, 'monotonic', return_value=100.0):
        launcher_bridge._poll()
        launcher_bridge._joint_command(command4(0.1, 0.0, 0.038, 0.6))
    assert launcher_port.servos[TRIGGER].goal == trigger_steps(module, 0.6)
    launcher_port.servos[RACK].present = rack_steps(module, 0.03)
    with patch.object(module.time, 'monotonic', return_value=100.5):
        launcher_bridge._poll()
    assert launcher_port.servos[TRIGGER].goal == 2048
    assert launcher_port.servos[RACK].goal == rack_steps(module, 0.03)
    launcher_bridge._joint_command(command4(0.1, 0.0, 0.038, 0.6))
    assert launcher_bridge._send_stop()  # destroy_node's STOP
    assert launcher_port.servos[TRIGGER].goal == 2048


def test_bridge_without_a_launcher_ignores_its_joints(module, bridge, port):
    port.servos[RACK], port.servos[TRIGGER] = Servo(present=512), Servo()
    bridge._poll()
    assert published_positions(bridge) == [[0.0, 0.0]]
    bridge._joint_command(command4(0.1, 0.0, 0.038, 0.6))
    assert port.servos[PAN].goal == pan_steps(module, 0.1)
    assert port.servos[RACK].goal_writes == 0 and port.servos[TRIGGER].goal_writes == 0
    assert bridge.ignored_actuator_commands == 1


def test_bridge_never_primes_or_fires_without_an_explicit_command(module, bridge,
                                                                  launcher_port):
    """Property: e-stop toggles, polls at any time, link failures and joint
    commands that do not ask for the launcher (absent, zero, negative or
    non-finite values) never move the trigger off 0 or the rack further back
    than it was."""
    bridge.actuator_fitted = True
    bridge.sts_config = turret_config(module, actuator=True)
    module.serial = NS(Serial=lambda path, baud, **options: launcher_port)
    rng = random.Random(14)
    safe_trigger = trigger_steps(module, 0.0)
    for _ in range(40):
        t = 100.0
        bridge.servos = None
        bridge.last_connect_attempt = float('-inf')
        launcher_port.is_open, launcher_port.fail = True, None
        launcher_port.servos[RACK].present = rng.choice((512, 1500, 2347))
        for _ in range(60):
            t += rng.choice((0.0, 0.01, 0.02, 0.3, 0.6))
            kind = rng.random()
            rack = launcher_port.servos[RACK]
            before = max(rack.present, rack.goal)
            with patch.object(module.time, 'monotonic', return_value=t):
                if kind < 0.3:
                    bridge._poll()
                elif kind < 0.4:
                    bridge._emergency_stop(NS(data=rng.random() < 0.5))
                elif kind < 0.45:
                    launcher_port.fail = OSError('unplugged') if rng.random() < 0.5 else None
                    bridge.last_connect_attempt = float('-inf')
                elif kind < 0.7:
                    bridge._joint_command(command4(rng.uniform(-2, 2), rng.uniform(-1, 1)))
                else:
                    value = rng.choice((0.0, -0.01, -1.0, math.nan, -math.inf))
                    bridge._joint_command(command4(rng.uniform(-2, 2), rng.uniform(-1, 1),
                                                   rng.choice((value, None)),
                                                   rng.choice((value, None))))
            assert launcher_port.servos[TRIGGER].goal <= safe_trigger
            assert launcher_port.servos[RACK].goal <= max(before, rack_steps(module, 0.0))
    assert launcher_port.servos[TRIGGER].goal_writes > 0  # the sequences did command it

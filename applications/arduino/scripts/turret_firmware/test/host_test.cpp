// Host unit tests for the turret firmware core (src/sts_servo.*, src/turret_core.*).
// Built and run by tests/test_turret_firmware.py:
//   c++ -std=c++11 -Wall -Wextra -Werror -Isrc test/host_test.cpp src/*.cpp -o host_test
//   ./host_test                      run the tests
//   ./host_test parse <line>         print how the firmware parses a host line
//   ./host_test format <pan> <tilt> [<actuator> <trigger>]
//                                    print the POS line the firmware sends
// arduino-cli ignores this folder (only the sketch root and src/ are built).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "sts_servo.h"
#include "turret_config.h"
#include "turret_core.h"

using turret::Command;
using turret::Config;
using turret::Controller;
namespace sts = turret::sts;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    ++g_checks;                                                            \
    if (!(cond)) {                                                         \
      ++g_failures;                                                        \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    }                                                                      \
  } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

typedef std::vector<uint8_t> Bytes;

static Bytes bytes(const uint8_t* data, size_t len) { return Bytes(data, data + len); }

static Bytes hex(const char* text) {
  Bytes out;
  while (*text != '\0') {
    char* end = NULL;
    const long value = std::strtol(text, &end, 16);
    if (end == text) {
      break;
    }
    out.push_back(static_cast<uint8_t>(value));
    text = end;
  }
  return out;
}

// Pan and tilt only (no launcher fitted), and the same with the launcher.
static const Config kConfig = {
    {
        {"pan_joint", 12, 2048, -1, -1.5708f, 1.5708f, 0.0f, 0},
        {"tilt_joint", 13, 2048, -1, -0.65f, 0.75f, 0.0f, 0},
        {"actuator_joint", 14, 512, 1, 0.0f, 0.038f, 0.0135f, 550},
        {"trigger_joint", 15, 2048, 1, 0.0f, 0.6f, 0.0f, 300},
    },
    2000, 50, 500, 40, 1000, 250, false, false,
};

static Config withLauncher() {
  Config config = kConfig;
  config.actuator_fitted = true;
  return config;
}

static const Config kLauncherConfig = withLauncher();

static const float kStep = static_cast<float>(2.0 * M_PI / 4096.0);

// ---- packets ----

static void testPackets() {
  uint8_t buf[sts::kMaxPacket];
  // Reference packets captured from Feetech's SCServo_Linux SMS_STS
  // (adityakamath/SCServo_Linux 79e70ff, the commit applications/pantiltros
  // builds) for the same calls, plus the classic examples of Feetech's manual.
  CHECK(bytes(buf, sts::buildPing(buf, sizeof(buf), 1)) == hex("FF FF 01 02 01 FB"));
  CHECK(bytes(buf, sts::buildRead(buf, sizeof(buf), 1, 56, 2)) == hex("FF FF 01 04 02 38 02 BE"));
  CHECK(bytes(buf, sts::buildRead(buf, sizeof(buf), 12, 56, 2)) == hex("FF FF 0C 04 02 38 02 B3"));
  const uint8_t on = 1;
  const uint8_t off = 0;
  CHECK(bytes(buf, sts::buildWrite(buf, sizeof(buf), 13, 40, &on, 1)) ==
        hex("FF FF 0D 04 03 28 01 C2"));
  CHECK(bytes(buf, sts::buildWrite(buf, sizeof(buf), 12, 40, &off, 1)) ==
        hex("FF FF 0C 04 03 28 00 C4"));
  // Goal position 2048, time 0, speed 1000 to ID 1 (registers 42-47)
  const uint8_t goal[6] = {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03};
  CHECK(bytes(buf, sts::buildWrite(buf, sizeof(buf), 1, 42, goal, 6)) ==
        hex("FF FF 01 09 03 2A 00 08 00 00 E8 03 D5"));

  const uint8_t ids[2] = {12, 13};
  const int32_t steps[2] = {1024, 3000};
  CHECK(bytes(buf, sts::buildSyncWritePosition(buf, sizeof(buf), ids, steps, 2, 2000, 50)) ==
        hex("FF FF FE 14 83 29 07 0C 32 00 04 00 00 D0 07 0D 32 B8 0B 00 00 D0 07 48"));
  const uint8_t ids_extreme[2] = {1, 2};
  const int32_t steps_extreme[2] = {0, 4095};
  CHECK(bytes(buf, sts::buildSyncWritePosition(buf, sizeof(buf), ids_extreme, steps_extreme, 2, 0,
                                               0)) ==
        hex("FF FF FE 14 83 29 07 01 00 00 00 00 00 00 00 02 00 FF 0F 00 00 00 00 29"));
  const uint8_t id_negative[1] = {7};
  const int32_t steps_negative[1] = {-100};
  CHECK(bytes(buf, sts::buildSyncWritePosition(buf, sizeof(buf), id_negative, steps_negative, 1, 500,
                                               10)) ==
        hex("FF FF FE 0C 83 29 07 07 0A 64 80 00 00 F4 01 58"));

  // The launcher's torque limits (registers 48-49, low byte first), the
  // rack's 55 % and the trigger's 30 %: computed per the protocol, not captured
  const uint8_t limit[2] = {0x26, 0x02};  // 550
  CHECK(bytes(buf, sts::buildWrite(buf, sizeof(buf), 14, sts::kRegTorqueLimit, limit, 2)) ==
        hex("FF FF 0E 05 03 30 26 02 91"));
  const uint8_t trigger_limit[2] = {0x2C, 0x01};  // 300
  CHECK(bytes(buf, sts::buildWrite(buf, sizeof(buf), 15, sts::kRegTorqueLimit, trigger_limit, 2)) ==
        hex("FF FF 0F 05 03 30 2C 01 8B"));

  // Checksum is ~(ID + LEN + INSTR + params), low byte
  const Bytes ping = hex("01 02 01");
  CHECK(sts::checksum(&ping[0], ping.size()) == 0xFB);

  // Too small a buffer
  CHECK(sts::buildPing(buf, 5, 1) == 0);
  CHECK(sts::buildSyncWritePosition(buf, sizeof(buf), ids, steps, 0, 0, 0) == 0);

  // Sign and magnitude positions
  CHECK(sts::encodePosition(0) == 0);
  CHECK(sts::encodePosition(4095) == 4095);
  CHECK(sts::encodePosition(-100) == (0x8000 | 100));
  CHECK(sts::decodePosition(0x8000 | 100) == -100);
  CHECK(sts::decodePosition(2048) == 2048);
}

static void testStatusParser() {
  sts::StatusParser parser;
  // Position 2048 from ID 12, with leading garbage and an extra header byte
  Bytes reply = hex("00 13 FF FF FF 0C 04 00 00 08");
  const uint8_t sum = sts::checksum(&reply[5], 5);
  reply.push_back(sum);
  bool done = false;
  for (size_t i = 0; i < reply.size(); ++i) {
    const bool complete = parser.feed(reply[i]);
    CHECK(!complete || i == reply.size() - 1);
    done = done || complete;
  }
  CHECK(done);
  CHECK(parser.id() == 12);
  CHECK(parser.error() == 0);
  CHECK(parser.numParams() == 2);
  CHECK(parser.params()[0] == 0x00 && parser.params()[1] == 0x08);

  // Bad checksum is dropped and counted
  sts::StatusParser bad;
  const Bytes corrupt = hex("FF FF 0C 04 00 00 08 00");
  bool any = false;
  for (size_t i = 0; i < corrupt.size(); ++i) {
    any = bad.feed(corrupt[i]) || any;
  }
  CHECK(!any);
  CHECK(bad.badPackets() == 1);

  // An echoed instruction packet is recognised as such
  uint8_t read[sts::kMaxPacket];
  const size_t len = sts::buildRead(read, sizeof(read), 12, 56, 2);
  sts::StatusParser echo;
  bool parsed = false;
  for (size_t i = 0; i < len; ++i) {
    parsed = echo.feed(read[i]) || parsed;
  }
  CHECK(parsed);
  CHECK(echo.equals(read, len));
}

// ---- conversions and limits ----

static void testConversions() {
  const turret::JointConfig& pan = kConfig.joints[turret::kPan];
  const turret::JointConfig& tilt = kConfig.joints[turret::kTilt];
  CHECK(turret::toSteps(pan, 0.0f) == 2048);
  // pan_tilt_ros convention: steps = 2048 - rad * 4096 / 2pi
  CHECK(turret::toSteps(pan, static_cast<float>(M_PI / 2)) == 1024);
  CHECK(turret::toSteps(pan, static_cast<float>(-M_PI / 2)) == 3072);
  CHECK(turret::toSteps(tilt, 0.75f) == 2048 - 489);
  CHECK(turret::toSteps(pan, 100.0f) == 0);     // clamped to the servo range
  CHECK(turret::toSteps(pan, -100.0f) == 4095);
  CHECK_NEAR(turret::fromSteps(pan, 1024), M_PI / 2, 1e-5);
  CHECK_NEAR(turret::fromSteps(pan, 2048), 0.0, 1e-9);
  for (int i = -200; i <= 200; ++i) {
    const float rad = i * 0.0077f;
    CHECK_NEAR(turret::fromSteps(pan, turret::toSteps(pan, rad)), rad, kStep / 2 + 1e-6);
  }
  turret::JointConfig forward = pan;
  forward.direction = 1;
  forward.zero_steps = 1000;
  CHECK(turret::toSteps(forward, 0.1f) == 1065);
  CHECK_NEAR(turret::fromSteps(forward, 1065), 0.1, kStep);

  // The rack: metres of travel over the pinion's pitch radius (13.5 mm)
  const turret::JointConfig& rack = kConfig.joints[turret::kActuator];
  CHECK(turret::toSteps(rack, 0.0f) == 512);
  CHECK(turret::toSteps(rack, 0.0135f) == 512 + 652);   // 1 rad of pinion
  CHECK(turret::toSteps(rack, 0.038f) == 512 + 1835);   // 2.81 rad: primed
  CHECK_NEAR(turret::fromSteps(rack, 512 + 652), 0.0135, 0.0135 * kStep);
  for (int i = 0; i <= 380; ++i) {
    const float m = i * 0.0001f;
    CHECK_NEAR(turret::fromSteps(rack, turret::toSteps(rack, m)), m, 0.0135 * kStep / 2 + 1e-7);
  }
  CHECK(turret::restPosition(rack) == 0.0f);
  CHECK(turret::clampToLimits(rack, 0.1f) == 0.038f);
  CHECK(turret::clampToLimits(rack, -0.01f) == 0.0f);
  const turret::JointConfig& trigger = kConfig.joints[turret::kTrigger];
  CHECK(turret::restPosition(trigger) == 0.0f);
  CHECK(turret::toSteps(trigger, 0.6f) == 2048 + 391);
  CHECK(turret::clampToLimits(trigger, 1.0f) == 0.6f);

  CHECK(turret::clampToLimits(tilt, 2.0f) == 0.75f);
  CHECK(turret::clampToLimits(tilt, -2.0f) == -0.65f);
  CHECK(turret::clampToLimits(tilt, 0.1f) == 0.1f);

  CHECK(turret::validateConfig(kConfig) == NULL);
  Config bad = kConfig;
  bad.joints[turret::kTilt].servo_id = 12;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].direction = 0;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].lower = 0.5f;
  bad.joints[turret::kPan].upper = 0.5f;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].upper = 3.5f;  // past step 0
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].lower = NAN;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.watchdog_ms = 0;
  CHECK(turret::validateConfig(bad) != NULL);

  // The shipped settings (src/turret_config.h) are usable, with the actuator too
  CHECK(turret::validateConfig(kTurretConfig) == NULL);
  Config shipped = kTurretConfig;
  shipped.actuator_fitted = true;
  CHECK(turret::validateConfig(shipped) == NULL);

  // The launcher is checked only when fitted
  CHECK(turret::validateConfig(kLauncherConfig) == NULL);
  bad = kConfig;
  bad.joints[turret::kActuator].servo_id = 12;
  CHECK(turret::validateConfig(bad) == NULL);
  bad.actuator_fitted = true;
  CHECK(turret::validateConfig(bad) != NULL);  // shares the pan servo's ID
  bad = kLauncherConfig;
  bad.joints[turret::kTrigger].servo_id = 14;  // shares the rack's
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kLauncherConfig;
  bad.joints[turret::kActuator].pinion_radius_m = 0.0f;  // a slide needs a radius
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kLauncherConfig;
  bad.joints[turret::kActuator].pinion_radius_m = 0.005f;  // 0.038 m is 7.6 rad: past one turn
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kLauncherConfig;
  bad.joints[turret::kActuator].pinion_radius_m = -0.01f;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kLauncherConfig;
  bad.joints[turret::kActuator].torque_limit = 1001;
  CHECK(turret::validateConfig(bad) != NULL);
}

// ---- line protocol ----

static std::string formatted(float value, int decimals) {
  char buf[40];
  turret::formatFixed(buf, sizeof(buf), value, decimals);
  return buf;
}

static void testLineProtocol() {
  Command c = turret::parseLine("SET pan_joint 0.500000 tilt_joint -0.250000");
  CHECK(c.type == Command::kSet);
  CHECK_NEAR(c.pan, 0.5, 1e-7);
  CHECK_NEAR(c.tilt, -0.25, 1e-7);
  c = turret::parseLine("SET tilt_joint 0.1 pan_joint -1.2");
  CHECK(c.type == Command::kSet);
  CHECK_NEAR(c.pan, -1.2, 1e-6);
  CHECK_NEAR(c.tilt, 0.1, 1e-6);
  c = turret::parseLine("  SET\tpan_joint 1e-3 tilt_joint 2\r");
  CHECK(c.type == Command::kSet);
  CHECK_NEAR(c.pan, 0.001, 1e-7);
  CHECK(!c.has_actuator && !c.has_trigger);
  c = turret::parseLine("SET pan_joint 0.1 tilt_joint -0.2 actuator_joint 0.070000 trigger_joint 0.600000");
  CHECK(c.type == Command::kSet && c.has_actuator && c.has_trigger);
  CHECK_NEAR(c.actuator, 0.07, 1e-7);
  CHECK_NEAR(c.trigger, 0.6, 1e-7);
  CHECK_NEAR(c.tilt, -0.2, 1e-7);
  c = turret::parseLine("SET actuator_joint 0 tilt_joint 0.1 pan_joint 0.2");
  CHECK(c.type == Command::kSet && c.has_actuator && c.actuator == 0.0f && !c.has_trigger);
  c = turret::parseLine("SET trigger_joint 0 tilt_joint 0.1 pan_joint 0.2");
  CHECK(c.type == Command::kSet && c.has_trigger && !c.has_actuator);
  CHECK(turret::parseLine("STOP").type == Command::kStop);
  CHECK(turret::parseLine("STOP\r").type == Command::kStop);
  CHECK(turret::parseLine("").type == Command::kNone);
  CHECK(turret::parseLine("   ").type == Command::kNone);
  const char* invalid[] = {
      "STOP now",
      "SET pan_joint 1 tilt_joint",
      "SET pan_joint 1 pan_joint 2",
      "SET pan_joint x tilt_joint 0",
      "SET pan_joint 1.0abc tilt_joint 0",
      "SET pan_joint nan tilt_joint 0",
      "SET pan_joint inf tilt_joint 0",
      "SET pan_joint 1 tilt_joint -inf",
      "SET elbow 1 tilt_joint 0",
      "SET pan_joint 1 tilt_joint 0 extra 1",
      "POS pan_joint 1 tilt_joint 0",
      "set pan_joint 1 tilt_joint 0",
      "SET pan_joint 1 actuator_joint 0.01",
      "SET pan_joint 1 tilt_joint 0 actuator_joint",
      "SET pan_joint 1 tilt_joint 0 actuator_joint nan",
      "SET pan_joint 1 tilt_joint 0 trigger_joint inf",
      "SET pan_joint 1 tilt_joint 0 actuator_joint 0.01 actuator_joint 0",
      "SET pan_joint 1 tilt_joint 0 trigger_joint 0.1 trigger_joint 0",
      "SET pan_joint 1 tilt_joint 0 actuator_joint 0.01 pan_joint 0",
      "SET pan_joint 1 tilt_joint 0 pan_joint 2 tilt_joint 3",
      "SET pan_joint 1 tilt_joint 0 actuator_joint 0 trigger_joint 0 extra 1",
  };
  for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
    const Command bad = turret::parseLine(invalid[i]);
    CHECK(bad.type == Command::kInvalid);
    CHECK(bad.error != NULL);
  }

  CHECK(formatted(0.0f, 6) == "0.000000");
  CHECK(formatted(1.5708f, 6) == "1.570800");
  CHECK(formatted(-0.65f, 6) == "-0.650000");
  CHECK(formatted(-0.0000001f, 6) == "0.000000");
  CHECK(formatted(12.5f, 0) == "13");
  CHECK(formatted(NAN, 6) == "nan");
  char line[80];
  turret::formatPosition(line, sizeof(line), 0.1f, -0.2f);
  CHECK(std::string(line) == "POS pan_joint 0.100000 tilt_joint -0.200000\n");
  char longer[128];
  turret::formatPosition(longer, sizeof(longer), 0.1f, -0.2f, true, 0.0695f, 0.6f);
  CHECK(std::string(longer) ==
        "POS pan_joint 0.100000 tilt_joint -0.200000 actuator_joint 0.069500 trigger_joint 0.600000\n");
  turret::formatPosition(longer, sizeof(longer), 0.1f, -0.2f, false, 0.0305f, 0.6f);
  CHECK(std::string(longer) == "POS pan_joint 0.100000 tilt_joint -0.200000\n");
  turret::formatError(line, sizeof(line), "servo 12 (pan_joint)\ntimeout");
  CHECK(std::string(line) == "ERR servo 12 (pan_joint) timeout\n");
  char tiny[8];
  turret::formatPosition(tiny, sizeof(tiny), 0.1f, -0.2f);
  CHECK(std::strlen(tiny) == sizeof(tiny) - 1);  // truncated, still terminated

  turret::LineReader reader;
  const char* input = "SET pan_joint 1 tilt_joint 2\r\nSTOP\n";
  std::vector<std::string> lines;
  for (const char* p = input; *p != '\0'; ++p) {
    if (reader.feed(*p)) {
      lines.push_back(reader.line());
    }
  }
  CHECK(lines.size() == 2);
  CHECK(lines.size() == 2 && lines[0] == "SET pan_joint 1 tilt_joint 2");
  CHECK(lines.size() == 2 && lines[1] == "STOP");
  std::string longline(turret::LineReader::kCapacity + 10, 'x');
  bool overflowed = false;
  for (size_t i = 0; i < longline.size(); ++i) {
    CHECK(!reader.feed(longline[i]));
  }
  CHECK(reader.feed('\n'));
  overflowed = reader.overflowed();
  CHECK(overflowed);
  CHECK(std::string(reader.line()).empty());
  const char* next = "STOP\n";
  bool got = false;
  for (const char* p = next; *p != '\0'; ++p) {
    got = reader.feed(*p) || got;
  }
  CHECK(got && !reader.overflowed() && std::string(reader.line()) == "STOP");
}

// ---- controller against simulated servos ----

struct SimServo {
  int32_t present;
  int32_t goal;
  bool torque;
  bool responds;
  uint8_t status;
  uint16_t speed;
  uint8_t acceleration;
  int goal_writes;
  uint16_t torque_limit;  // 1000 at power-up
};

// Two STS servos behind a byte link: decodes the instruction packets the bus
// sends and queues their status replies (optionally after an echo).
class SimLink : public sts::ByteLink {
 public:
  SimLink() : echo(false), now_us(0) {}

  void write(const uint8_t* data, size_t len) override {
    const Bytes packet(data, data + len);
    sent.push_back(packet);
    if (echo) {
      rx.insert(rx.end(), packet.begin(), packet.end());
    }
    handle(packet);
  }
  int read() override {
    now_us += 10;
    if (rx.empty()) {
      return -1;
    }
    const uint8_t byte = rx.front();
    rx.erase(rx.begin());
    return byte;
  }
  uint32_t micros() override { return now_us; }

  std::map<uint8_t, SimServo> servos;
  std::vector<Bytes> sent;
  Bytes rx;
  bool echo;
  uint32_t now_us;
  int bad_checksums = 0;

  size_t countInstruction(uint8_t instruction) const {
    size_t n = 0;
    for (size_t i = 0; i < sent.size(); ++i) {
      n += sent[i].size() > 4 && sent[i][4] == instruction;
    }
    return n;
  }

 private:
  void reply(uint8_t id, const Bytes& params) {
    SimServo& servo = servos[id];
    if (!servo.responds) {
      return;
    }
    Bytes packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(static_cast<uint8_t>(params.size() + 2));
    packet.push_back(servo.status);
    packet.insert(packet.end(), params.begin(), params.end());
    packet.push_back(sts::checksum(&packet[2], packet.size() - 2));
    rx.insert(rx.end(), packet.begin(), packet.end());
  }

  void handle(const Bytes& p) {
    if (p.size() < 6 || p[0] != 0xFF || p[1] != 0xFF || p[3] + 4u != p.size() ||
        sts::checksum(&p[2], p.size() - 3) != p.back()) {
      ++bad_checksums;
      return;
    }
    const uint8_t id = p[2];
    const uint8_t instruction = p[4];
    if (instruction == sts::kInstRead && p[5] == sts::kRegPresentPosition && p[6] == 2) {
      if (servos.count(id) == 0) {
        return;
      }
      const uint16_t raw = sts::encodePosition(servos[id].present);
      Bytes params;
      params.push_back(static_cast<uint8_t>(raw & 0xFF));
      params.push_back(static_cast<uint8_t>(raw >> 8));
      reply(id, params);
    } else if (instruction == sts::kInstWrite && p[5] == sts::kRegTorqueLimit && p.size() == 9) {
      if (servos.count(id) == 0) {
        return;
      }
      if (servos[id].responds) {
        servos[id].torque_limit = static_cast<uint16_t>(p[6] | (p[7] << 8));
      }
      reply(id, Bytes());
    } else if (instruction == sts::kInstWrite && p[5] == sts::kRegTorqueEnable) {
      if (servos.count(id) == 0) {
        return;
      }
      if (servos[id].responds) {
        servos[id].torque = p[6] != 0;
      }
      reply(id, Bytes());
    } else if (instruction == sts::kInstSyncWrite && id == sts::kBroadcastId) {
      const uint8_t per_servo = p[6];
      for (size_t at = 7; at + per_servo + 1 < p.size(); at += per_servo + 1) {
        if (servos.count(p[at]) == 0 || !servos[p[at]].responds) {
          continue;
        }
        SimServo& servo = servos[p[at]];
        servo.acceleration = p[at + 1];
        servo.goal = sts::decodePosition(static_cast<uint16_t>(p[at + 2] | (p[at + 3] << 8)));
        servo.speed = static_cast<uint16_t>(p[at + 6] | (p[at + 7] << 8));
        ++servo.goal_writes;
      }
    }
  }
};

class RecordingOutput : public turret::Output {
 public:
  void position(float pan, float tilt, bool has_launcher, float actuator,
                float trigger) override {
    positions.push_back(std::make_pair(pan, tilt));
    actuators.push_back(has_launcher ? actuator : NAN);  // NAN: not reported
    triggers.push_back(has_launcher ? trigger : NAN);
  }
  void error(const char* text) override { errors.push_back(text); }
  std::vector<std::pair<float, float> > positions;
  std::vector<float> actuators;
  std::vector<float> triggers;
  std::vector<std::string> errors;

  size_t errorsContaining(const char* needle) const {
    size_t n = 0;
    for (size_t i = 0; i < errors.size(); ++i) {
      n += errors[i].find(needle) != std::string::npos;
    }
    return n;
  }
};

struct Rig {
  explicit Rig(const Config& config = kConfig)
      : bus(link, 3000), controller(config, bus, output) {
    SimServo servo = {2048, 2048, false, true, 0, 0, 0, 0, 1000};
    link.servos[12] = servo;
    link.servos[13] = servo;
    if (config.actuator_fitted) {
      SimServo rack = {512, 512, false, true, 0, 0, 0, 0, 1000};  // forward
      link.servos[14] = rack;
      link.servos[15] = servo;  // trigger at 0 (safe)
    }
  }
  SimLink link;
  sts::Bus bus;
  RecordingOutput output;
  Controller controller;
};

static void testControllerCommands() {
  Rig rig;
  rig.link.servos[12].present = 1024;  // pan at +pi/2
  rig.link.servos[13].present = 2048;

  // Idle: feedback only, nothing written to the servos
  rig.controller.update(0);
  CHECK(rig.output.positions.empty());  // first report after one period
  rig.controller.update(40);
  CHECK(rig.output.positions.size() == 1);
  if (!rig.output.positions.empty()) {
    CHECK_NEAR(rig.output.positions[0].first, M_PI / 2, 1e-4);
    CHECK_NEAR(rig.output.positions[0].second, 0.0, 1e-6);
  }
  CHECK(rig.link.countInstruction(sts::kInstSyncWrite) == 0);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 0);
  CHECK(rig.controller.state() == Controller::kIdle);
  float pan = 0.0f;
  float tilt = 0.0f;
  CHECK(rig.controller.measured(pan, tilt));

  // First SET enables torque and moves both servos in one sync write
  CHECK(rig.controller.set(0.5f, -0.25f, 50));
  CHECK(rig.controller.state() == Controller::kActive);
  CHECK(rig.link.servos[12].torque && rig.link.servos[13].torque);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 2);
  CHECK(rig.link.servos[12].goal == turret::toSteps(kConfig.joints[0], 0.5f));
  CHECK(rig.link.servos[13].goal == turret::toSteps(kConfig.joints[1], -0.25f));
  CHECK(rig.link.servos[12].speed == 2000 && rig.link.servos[12].acceleration == 50);

  // Later SETs only write goals; out-of-range goals are clamped to the limits
  CHECK(rig.controller.set(3.0f, 2.0f, 60));
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 2);
  CHECK(rig.link.servos[12].goal == turret::toSteps(kConfig.joints[0], 1.5708f));
  CHECK(rig.link.servos[13].goal == turret::toSteps(kConfig.joints[1], 0.75f));
  CHECK(rig.controller.goal(turret::kTilt) == 0.75f);
  CHECK(rig.controller.set(-3.0f, -2.0f, 70));
  CHECK(rig.link.servos[13].goal == turret::toSteps(kConfig.joints[1], -0.65f));

  // Non-finite values are rejected with an error and move nothing
  const int writes = rig.link.servos[12].goal_writes;
  CHECK(!rig.controller.set(NAN, 0.0f, 80));
  CHECK(!rig.controller.set(0.0f, INFINITY, 80));
  CHECK(rig.link.servos[12].goal_writes == writes);
  CHECK(rig.output.errorsContaining("non-finite") == 2);
  CHECK(rig.link.bad_checksums == 0);
}

static void testWatchdogAndStop() {
  Rig rig;
  rig.controller.set(0.2f, 0.1f, 1000);
  rig.link.servos[12].present = 1900;  // still on its way
  rig.link.servos[13].present = 2000;
  const int writes = rig.link.servos[12].goal_writes;
  rig.controller.update(1499);
  CHECK(rig.controller.state() == Controller::kActive);
  CHECK(rig.link.servos[12].goal_writes == writes);
  // 500 ms without SET: hold where the servos are
  rig.controller.update(1500);
  CHECK(rig.controller.state() == Controller::kHolding);
  CHECK(rig.link.servos[12].goal == 1900);
  CHECK(rig.link.servos[13].goal == 2000);
  CHECK(rig.link.servos[12].torque);  // holding, not limp
  CHECK(rig.output.errors.empty());   // the watchdog is normal operation
  // No further goals until the next SET
  const int held = rig.link.servos[12].goal_writes;
  for (uint32_t t = 1500; t < 3000; t += 10) {
    rig.controller.update(t);
  }
  CHECK(rig.link.servos[12].goal_writes == held);

  // A SET resumes (re-enabling torque), STOP holds again
  const size_t torque_writes = rig.link.countInstruction(sts::kInstWrite);
  CHECK(rig.controller.set(-0.3f, 0.0f, 3000));
  CHECK(rig.controller.state() == Controller::kActive);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == torque_writes + 2);
  rig.link.servos[12].present = 2100;
  rig.controller.stop(3010);
  CHECK(rig.controller.state() == Controller::kHolding);
  CHECK(rig.link.servos[12].goal == 2100);

  // STOP before any SET is harmless: it holds the present position
  Rig idle;
  idle.link.servos[12].present = 1500;
  idle.controller.stop(5);
  CHECK(idle.link.servos[12].goal == 1500);
  CHECK(idle.controller.state() == Controller::kHolding);

  // Optional torque-off stop
  Config limp = kConfig;
  limp.stop_disables_torque = true;
  Rig rig2(limp);
  rig2.controller.set(0.0f, 0.0f, 0);
  CHECK(rig2.link.servos[12].torque);
  rig2.controller.stop(10);
  CHECK(!rig2.link.servos[12].torque && !rig2.link.servos[13].torque);
  CHECK(rig2.controller.set(0.1f, 0.0f, 20));
  CHECK(rig2.link.servos[12].torque && rig2.link.servos[13].torque);

  // millis() wrap-around
  Rig wrap;
  wrap.controller.set(0.0f, 0.0f, 0xFFFFFF00u);
  wrap.controller.update(0xFFFFFF00u + 499u);
  CHECK(wrap.controller.state() == Controller::kActive);
  wrap.controller.update(0xFFFFFF00u + 500u);  // wraps past zero
  CHECK(wrap.controller.state() == Controller::kHolding);
}

static void testServoErrors() {
  Rig rig;
  rig.link.servos[13].responds = false;
  rig.controller.update(40);
  CHECK(rig.output.positions.empty());  // no POS while a joint is not measured
  CHECK(rig.output.errors.size() == 1);
  CHECK(rig.output.errors.size() == 1 &&
        rig.output.errors[0] == "servo 13 (tilt_joint) timeout");
  float pan = 0.0f;
  float tilt = 0.0f;
  CHECK(!rig.controller.measured(pan, tilt));
  // Repeated at most once per error_repeat_ms
  for (uint32_t t = 80; t < 1040; t += 40) {
    rig.controller.update(t);
  }
  CHECK(rig.output.errorsContaining("timeout") == 1);
  rig.controller.update(1040);
  CHECK(rig.output.errorsContaining("timeout") == 2);

  // Recovery: POS again, and the next failure is reported at once
  rig.link.servos[13].responds = true;
  rig.controller.update(1080);
  CHECK(rig.output.positions.size() == 1);
  rig.link.servos[13].responds = false;
  rig.controller.update(1120);
  CHECK(rig.output.errorsContaining("timeout") == 3);

  // Torque enable failure is reported; a SET still writes goals
  rig.controller.set(0.1f, 0.1f, 1130);
  CHECK(rig.output.errorsContaining("torque enable timeout") <= 1);
  CHECK(rig.link.servos[12].torque);
  // While the servo stays silent, torque re-enable is retried at most every 250 ms
  const size_t writes = rig.link.countInstruction(sts::kInstWrite);
  rig.controller.set(0.1f, 0.1f, 1140);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == writes);
  rig.controller.set(0.1f, 0.1f, 1400);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == writes + 2);
  // STOP cannot hold a joint that does not answer, and says so
  rig.controller.stop(1500);
  CHECK(rig.output.errorsContaining("cannot hold") == 1);

  // Servo fault bits are reported, but the position is still valid
  Rig fault;
  fault.link.servos[12].status = 0x20;
  fault.controller.update(40);
  CHECK(fault.output.positions.size() == 1);
  CHECK(fault.output.errorsContaining("servo 12 (pan_joint) fault 0x20 overload") == 1);

  // Invalid configuration: errors, never moves
  Config bad = kConfig;
  bad.joints[turret::kPan].direction = 0;
  Rig broken(bad);
  CHECK(!broken.controller.set(0.0f, 0.0f, 0));
  broken.controller.update(10);
  broken.controller.stop(20);
  CHECK(broken.link.sent.empty());
  CHECK(broken.output.errorsContaining("invalid firmware config") == 2);
}

static void testEchoingAdapter() {
  // A one-wire adapter that loops TX back to RX
  Rig rig;
  rig.link.echo = true;
  rig.link.servos[12].present = 3000;
  rig.controller.update(40);
  CHECK(rig.output.errors.empty());
  CHECK(rig.output.positions.size() == 1);
  if (!rig.output.positions.empty()) {
    CHECK_NEAR(rig.output.positions[0].first, turret::fromSteps(kConfig.joints[0], 3000), 1e-6);
  }
  CHECK(rig.controller.set(0.0f, 0.0f, 50));
  CHECK(rig.output.errors.empty());
  CHECK(rig.link.servos[12].torque);
  rig.link.servos[13].responds = false;
  rig.controller.update(90);
  CHECK(rig.output.errorsContaining("servo 13 (tilt_joint) timeout") == 1);
}

// ---- the dart launcher ----

static int32_t rackSteps(float m) { return turret::toSteps(kLauncherConfig.joints[turret::kActuator], m); }
static int32_t triggerSteps(float rad) {
  return turret::toSteps(kLauncherConfig.joints[turret::kTrigger], rad);
}

static void testLauncherCommands() {
  Rig rig(kLauncherConfig);
  CHECK(rig.controller.numJoints() == 4);
  // Idle: POS carries the measured launcher, nothing is written
  rig.link.servos[14].present = rackSteps(0.002f);
  rig.controller.update(40);
  CHECK(rig.output.positions.size() == 1);
  CHECK(rig.output.actuators.size() == 1 && std::fabs(rig.output.actuators[0] - 0.002f) < 1e-4f);
  CHECK(rig.output.triggers.size() == 1 && std::fabs(rig.output.triggers[0]) < 1e-4f);
  float actuator = 1.0f;
  float trigger = 1.0f;
  CHECK(rig.controller.measuredLauncher(actuator, trigger) && std::fabs(actuator - 0.002f) < 1e-4f);
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 0);
  CHECK(rig.link.countInstruction(sts::kInstSyncWrite) == 0);

  // A SET naming both: torque on for all four (and the rack's torque limit
  // of 55 %, the trigger's of 30 %), one sync write
  CHECK(rig.controller.set(0.1f, 0.0f, 0.038f, 0.0f, 50));
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 6);
  CHECK(rig.link.servos[14].torque && rig.link.servos[15].torque);
  CHECK(rig.link.servos[14].torque_limit == 550 && rig.link.servos[15].torque_limit == 300);
  CHECK(rig.link.servos[12].torque_limit == 1000);
  CHECK(rig.link.countInstruction(sts::kInstSyncWrite) == 1);
  CHECK(rig.link.servos[14].goal == rackSteps(0.038f));
  CHECK(rig.link.servos[15].goal == triggerSteps(0.0f));
  CHECK(rig.link.servos[12].goal == turret::toSteps(kConfig.joints[0], 0.1f));
  // Clamped to the limits
  CHECK(rig.controller.set(0.1f, 0.0f, 0.5f, 2.0f, 60));
  CHECK(rig.link.servos[14].goal == rackSteps(0.038f) && rig.link.servos[15].goal == triggerSteps(0.6f));
  CHECK(rig.controller.set(0.1f, 0.0f, -0.5f, -1.0f, 70));
  CHECK(rig.link.servos[14].goal == rackSteps(0.0f) && rig.link.servos[15].goal == triggerSteps(0.0f));
  // A SET that does not name the trigger returns it to 0; one that does not
  // name the rack stops it where it is (its present position)
  CHECK(rig.controller.set(0.1f, 0.0f, 0.020f, 0.6f, 80));
  rig.link.servos[14].present = rackSteps(0.017f);  // on its way back
  CHECK(rig.controller.set(0.1f, 0.0f, 90));
  CHECK(rig.link.servos[15].goal == triggerSteps(0.0f) && rig.controller.goal(turret::kTrigger) == 0.0f);
  CHECK(rig.link.servos[14].goal == rackSteps(0.017f));
  CHECK(rig.controller.setJoints(0.1f, 0.0f, false, 0.0f, true, 0.3f, 95));
  CHECK(rig.link.servos[15].goal == triggerSteps(0.3f) && rig.link.servos[14].goal == rackSteps(0.017f));
  CHECK(rig.controller.setJoints(0.1f, 0.0f, true, 0.01f, false, 0.0f, 98));
  CHECK(rig.link.servos[14].goal == rackSteps(0.01f) && rig.link.servos[15].goal == triggerSteps(0.0f));
  // Non-finite launcher values are rejected and move nothing
  const int writes = rig.link.servos[14].goal_writes;
  CHECK(!rig.controller.set(0.1f, 0.0f, NAN, 0.0f, 100));
  CHECK(!rig.controller.set(0.1f, 0.0f, 0.0f, INFINITY, 100));
  CHECK(rig.link.servos[14].goal_writes == writes);
  CHECK(rig.output.errorsContaining("non-finite") == 2);
  CHECK(rig.link.bad_checksums == 0);
}

static void testLauncherSafeState() {
  // Watchdog: pan/tilt hold where they are, the rack stops where it is, the
  // trigger returns to 0
  Rig rig(kLauncherConfig);
  rig.controller.set(0.2f, 0.1f, 0.038f, 0.6f, 1000);
  rig.link.servos[12].present = 1900;
  rig.link.servos[13].present = 2000;
  rig.link.servos[14].present = rackSteps(0.017f);  // on its way back
  rig.link.servos[15].present = triggerSteps(0.3f);
  rig.controller.update(1499);
  CHECK(rig.link.servos[14].goal == rackSteps(0.038f) && rig.link.servos[15].goal == triggerSteps(0.6f));
  rig.controller.update(1500);
  CHECK(rig.controller.state() == Controller::kHolding);
  CHECK(rig.link.servos[12].goal == 1900 && rig.link.servos[13].goal == 2000);
  CHECK(rig.link.servos[14].goal == rackSteps(0.017f));
  CHECK(rig.link.servos[15].goal == triggerSteps(0.0f));
  CHECK(rig.controller.goal(turret::kTrigger) == 0.0f);
  CHECK(rig.output.errors.empty());

  // STOP: the same
  rig.controller.set(0.2f, 0.1f, 0.038f, 0.6f, 2000);
  rig.link.servos[14].present = rackSteps(0.025f);
  rig.controller.stop(2010);
  CHECK(rig.link.servos[14].goal == rackSteps(0.025f) && rig.link.servos[15].goal == triggerSteps(0.0f));
  CHECK(rig.controller.state() == Controller::kHolding);

  // STOP before any SET: the trigger to 0, the rack held, torque untouched
  Rig idle(kLauncherConfig);
  idle.link.servos[14].present = rackSteps(0.03f);
  idle.link.servos[15].present = triggerSteps(0.5f);
  idle.controller.stop(5);
  CHECK(idle.link.servos[14].goal == rackSteps(0.03f) && idle.link.servos[15].goal == triggerSteps(0.0f));
  CHECK(idle.link.countInstruction(sts::kInstWrite) == 0);

  // stop_disables_torque applies to pan and tilt; the launcher stays safe and powered
  Config limp = kLauncherConfig;
  limp.stop_disables_torque = true;
  Rig rig2(limp);
  rig2.controller.set(0.0f, 0.0f, 0.038f, 0.6f, 0);
  rig2.link.servos[14].present = rackSteps(0.02f);
  rig2.controller.stop(10);
  CHECK(!rig2.link.servos[12].torque && !rig2.link.servos[13].torque);
  CHECK(rig2.link.servos[14].torque && rig2.link.servos[15].torque);
  CHECK(rig2.link.servos[14].goal == rackSteps(0.02f) && rig2.link.servos[15].goal == triggerSteps(0.0f));

  // A silent rack cannot be held, and says so; the trigger still returns to 0
  Rig silent(kLauncherConfig);
  silent.controller.set(0.0f, 0.0f, 0.038f, 0.6f, 0);
  silent.link.servos[14].responds = false;
  silent.controller.stop(10);
  CHECK(silent.link.servos[15].goal == triggerSteps(0.0f));
  CHECK(silent.output.errorsContaining("servo 14 (actuator_joint) cannot hold") == 1);
}

static void testLauncherErrors() {
  // A missing launcher servo: reported like the others, pan/tilt still flow
  Rig rig(kLauncherConfig);
  rig.link.servos[15].responds = false;
  rig.controller.update(40);
  CHECK(rig.output.positions.size() == 1);
  CHECK(rig.output.actuators.size() == 1 && std::isnan(rig.output.actuators[0]));
  CHECK(rig.output.errors.size() == 1 &&
        rig.output.errors[0] == "servo 15 (trigger_joint) timeout");
  float actuator = 0.0f;
  float trigger = 0.0f;
  CHECK(!rig.controller.measuredLauncher(actuator, trigger));
  float pan = 0.0f;
  float tilt = 0.0f;
  CHECK(rig.controller.measured(pan, tilt));
  for (uint32_t t = 80; t < 1040; t += 40) {
    rig.controller.update(t);
  }
  CHECK(rig.output.errorsContaining("servo 15 (trigger_joint) timeout") == 1);
  rig.controller.update(1040);
  CHECK(rig.output.errorsContaining("servo 15 (trigger_joint) timeout") == 2);
  rig.link.servos[15].responds = true;
  rig.controller.update(1080);
  CHECK(!std::isnan(rig.output.actuators.back()) && !std::isnan(rig.output.triggers.back()));
  // A silent rack at the first SET: torque enable reported, goals still go out
  Rig rack(kLauncherConfig);
  rack.link.servos[14].responds = false;
  rack.controller.set(0.0f, 0.0f, 0.01f, 0.0f, 0);
  CHECK(rack.output.errorsContaining("servo 14 (actuator_joint) torque enable timeout") == 1);
  CHECK(rack.link.servos[15].goal == triggerSteps(0.0f));

  // Fault bits are reported and the position is still valid
  Rig fault(kLauncherConfig);
  fault.link.servos[14].status = 0x20;
  fault.controller.update(40);
  CHECK(fault.output.positions.size() == 1 && !std::isnan(fault.output.actuators[0]));
  CHECK(fault.output.errorsContaining("servo 14 (actuator_joint) fault 0x20 overload") == 1);

  // Without a launcher fitted, its joints are ignored with an error at most
  // once per error_repeat_ms; pan and tilt still move, servos 14 and 15 never
  Rig plain;
  SimServo rest = {512, 512, false, true, 0, 0, 0, 0, 1000};
  plain.link.servos[14] = rest;
  plain.link.servos[15] = rest;
  CHECK(plain.controller.numJoints() == 2);
  CHECK(plain.controller.set(0.3f, 0.0f, 0.038f, 0.6f, 0));
  CHECK(plain.link.servos[12].goal == turret::toSteps(kConfig.joints[0], 0.3f));
  for (uint32_t t = 20; t < 1000; t += 20) {
    plain.controller.set(0.3f, 0.0f, 0.038f, 0.6f, t);
  }
  CHECK(plain.output.errorsContaining("ignored: no launcher fitted") == 1);
  plain.controller.set(0.3f, 0.0f, 0.038f, 0.6f, 1000);
  CHECK(plain.output.errorsContaining("ignored: no launcher fitted") == 2);
  plain.controller.update(1040);
  plain.controller.stop(1050);
  CHECK(!plain.output.positions.empty() && std::isnan(plain.output.actuators.back()));
  CHECK(plain.link.servos[14].goal_writes == 0 && plain.link.servos[15].goal_writes == 0);
  CHECK(!plain.link.servos[14].torque && !plain.link.servos[15].torque);
}

static int runTests() {
  testPackets();
  testStatusParser();
  testConversions();
  testLineProtocol();
  testControllerCommands();
  testWatchdogAndStop();
  testServoErrors();
  testEchoingAdapter();
  testLauncherCommands();
  testLauncherSafeState();
  testLauncherErrors();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "parse") == 0) {
    const Command c = turret::parseLine(argv[2]);
    switch (c.type) {
      case Command::kSet:
        std::printf("SET %.6f %.6f", c.pan, c.tilt);
        if (c.has_actuator) {
          std::printf(" actuator %.6f", c.actuator);
        }
        if (c.has_trigger) {
          std::printf(" trigger %.6f", c.trigger);
        }
        std::printf("\n");
        break;
      case Command::kStop:
        std::printf("STOP\n");
        break;
      case Command::kNone:
        std::printf("NONE\n");
        break;
      case Command::kInvalid:
        std::printf("INVALID %s\n", c.error);
        break;
    }
    return 0;
  }
  if ((argc == 4 || argc == 6) && std::strcmp(argv[1], "format") == 0) {
    char line[128];
    turret::formatPosition(line, sizeof(line), static_cast<float>(std::atof(argv[2])),
                           static_cast<float>(std::atof(argv[3])), argc == 6,
                           argc == 6 ? static_cast<float>(std::atof(argv[4])) : 0.0f,
                           argc == 6 ? static_cast<float>(std::atof(argv[5])) : 0.0f);
    std::fputs(line, stdout);
    return 0;
  }
  return runTests();
}

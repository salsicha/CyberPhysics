// Host unit tests for the turret firmware core (src/sts_servo.*, src/turret_core.*).
// Built and run by tests/test_turret_firmware.py:
//   c++ -std=c++11 -Wall -Wextra -Werror -Isrc test/host_test.cpp src/*.cpp -o host_test
//   ./host_test                      run the tests
//   ./host_test parse <line>         print how the firmware parses a host line
//   ./host_test format <pan> <tilt>  print the POS line the firmware sends
// arduino-cli ignores this folder (only the sketch root and src/ are built).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "sts_servo.h"
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

static const Config kConfig = {
    {
        {"pan_joint", 12, 2048, -1, -1.5708f, 1.5708f},
        {"tilt_joint", 13, 2048, -1, -0.65f, 0.75f},
    },
    2000, 50, 500, 40, 1000, 250, false,
};

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
  CHECK(turret::radToSteps(pan, 0.0f) == 2048);
  // pan_tilt_ros convention: steps = 2048 - rad * 4096 / 2pi
  CHECK(turret::radToSteps(pan, static_cast<float>(M_PI / 2)) == 1024);
  CHECK(turret::radToSteps(pan, static_cast<float>(-M_PI / 2)) == 3072);
  CHECK(turret::radToSteps(tilt, 0.75f) == 2048 - 489);
  CHECK(turret::radToSteps(pan, 100.0f) == 0);     // clamped to the servo range
  CHECK(turret::radToSteps(pan, -100.0f) == 4095);
  CHECK_NEAR(turret::stepsToRad(pan, 1024), M_PI / 2, 1e-5);
  CHECK_NEAR(turret::stepsToRad(pan, 2048), 0.0, 1e-9);
  for (int i = -200; i <= 200; ++i) {
    const float rad = i * 0.0077f;
    CHECK_NEAR(turret::stepsToRad(pan, turret::radToSteps(pan, rad)), rad, kStep / 2 + 1e-6);
  }
  turret::JointConfig forward = pan;
  forward.direction = 1;
  forward.zero_steps = 1000;
  CHECK(turret::radToSteps(forward, 0.1f) == 1065);
  CHECK_NEAR(turret::stepsToRad(forward, 1065), 0.1, kStep);

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
  bad.joints[turret::kPan].min_rad = 0.5f;
  bad.joints[turret::kPan].max_rad = 0.5f;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].max_rad = 3.5f;  // past step 0
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.joints[turret::kPan].min_rad = NAN;
  CHECK(turret::validateConfig(bad) != NULL);
  bad = kConfig;
  bad.watchdog_ms = 0;
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
  void position(float pan, float tilt) override {
    positions.push_back(std::make_pair(pan, tilt));
  }
  void error(const char* text) override { errors.push_back(text); }
  std::vector<std::pair<float, float> > positions;
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
    SimServo servo = {2048, 2048, false, true, 0, 0, 0, 0};
    link.servos[12] = servo;
    link.servos[13] = servo;
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
  CHECK(rig.link.servos[12].goal == turret::radToSteps(kConfig.joints[0], 0.5f));
  CHECK(rig.link.servos[13].goal == turret::radToSteps(kConfig.joints[1], -0.25f));
  CHECK(rig.link.servos[12].speed == 2000 && rig.link.servos[12].acceleration == 50);

  // Later SETs only write goals; out-of-range goals are clamped to the limits
  CHECK(rig.controller.set(3.0f, 2.0f, 60));
  CHECK(rig.link.countInstruction(sts::kInstWrite) == 2);
  CHECK(rig.link.servos[12].goal == turret::radToSteps(kConfig.joints[0], 1.5708f));
  CHECK(rig.link.servos[13].goal == turret::radToSteps(kConfig.joints[1], 0.75f));
  CHECK(rig.controller.goal(turret::kTilt) == 0.75f);
  CHECK(rig.controller.set(-3.0f, -2.0f, 70));
  CHECK(rig.link.servos[13].goal == turret::radToSteps(kConfig.joints[1], -0.65f));

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
    CHECK_NEAR(rig.output.positions[0].first, turret::stepsToRad(kConfig.joints[0], 3000), 1e-6);
  }
  CHECK(rig.controller.set(0.0f, 0.0f, 50));
  CHECK(rig.output.errors.empty());
  CHECK(rig.link.servos[12].torque);
  rig.link.servos[13].responds = false;
  rig.controller.update(90);
  CHECK(rig.output.errorsContaining("servo 13 (tilt_joint) timeout") == 1);
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
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "parse") == 0) {
    const Command c = turret::parseLine(argv[2]);
    switch (c.type) {
      case Command::kSet:
        std::printf("SET %.6f %.6f\n", c.pan, c.tilt);
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
  if (argc == 4 && std::strcmp(argv[1], "format") == 0) {
    char line[80];
    turret::formatPosition(line, sizeof(line), static_cast<float>(std::atof(argv[2])),
                           static_cast<float>(std::atof(argv[3])));
    std::fputs(line, stdout);
    return 0;
  }
  return runTests();
}

#include "turret_core.h"

#include <cmath>
#include <stdlib.h>
#include <string.h>

namespace turret {

namespace {

const float kStepsPerRad = static_cast<float>(sts::kStepsPerTurn / (2.0 * 3.14159265358979323846));

// Bounded string builder (no printf on the targets)
class Text {
 public:
  Text(char* out, size_t cap) : out_(out), cap_(cap), len_(0) {
    if (cap_ > 0) {
      out_[0] = '\0';
    }
  }
  Text& add(const char* s) {
    while (*s != '\0') {
      put(*s++);
    }
    return *this;
  }
  Text& addUint(unsigned long value) {
    char digits[12];
    size_t n = 0;
    do {
      digits[n++] = static_cast<char>('0' + value % 10);
      value /= 10;
    } while (value != 0 && n < sizeof(digits));
    while (n > 0) {
      put(digits[--n]);
    }
    return *this;
  }
  Text& addHex2(uint8_t value) {
    static const char kHex[] = "0123456789ABCDEF";
    put('0');
    put('x');
    put(kHex[value >> 4]);
    put(kHex[value & 0x0F]);
    return *this;
  }
  Text& addFixed(float value, int decimals) {
    len_ += formatFixed(out_ + len_, cap_ - len_, value, decimals);
    return *this;
  }
  size_t length() const { return len_; }

 private:
  void put(char c) {
    if (len_ + 1 < cap_) {
      out_[len_++] = c;
      out_[len_] = '\0';
    }
  }
  char* out_;
  size_t cap_;
  size_t len_;
};

bool isFiniteValue(float value) { return std::isfinite(value); }

}  // namespace

// ---- configuration and conversions ----

const char* validateConfig(const Config& config) {
  for (size_t j = 0; j < kNumJoints; ++j) {
    const JointConfig& joint = config.joints[j];
    if (joint.direction != 1 && joint.direction != -1) {
      return "direction must be +1 or -1";
    }
    if (joint.servo_id > 253) {
      return "servo id must be 0-253";
    }
    if (joint.zero_steps < 0 || joint.zero_steps > sts::kMaxStep) {
      return "zero_steps must be 0-4095";
    }
    if (!isFiniteValue(joint.min_rad) || !isFiniteValue(joint.max_rad) || !(joint.min_rad < joint.max_rad)) {
      return "joint limits must be finite with min < max";
    }
    // The limits must map inside one servo turn, or clamping to 0..4095
    // would silently move the joint somewhere else.
    const float a = joint.zero_steps + joint.direction * joint.min_rad * kStepsPerRad;
    const float b = joint.zero_steps + joint.direction * joint.max_rad * kStepsPerRad;
    if (a < -0.5f || a > sts::kMaxStep + 0.5f || b < -0.5f || b > sts::kMaxStep + 0.5f) {
      return "joint limits reach outside servo steps 0-4095";
    }
  }
  if (config.joints[kPan].servo_id == config.joints[kTilt].servo_id) {
    return "pan and tilt servo ids must differ";
  }
  if (config.watchdog_ms == 0 || config.feedback_period_ms == 0) {
    return "watchdog_ms and feedback_period_ms must be > 0";
  }
  return NULL;
}

float clampToLimits(const JointConfig& joint, float rad) {
  if (rad < joint.min_rad) {
    return joint.min_rad;
  }
  if (rad > joint.max_rad) {
    return joint.max_rad;
  }
  return rad;
}

int32_t radToSteps(const JointConfig& joint, float rad) {
  const float steps = joint.zero_steps + joint.direction * rad * kStepsPerRad;
  int32_t rounded = static_cast<int32_t>(std::floor(steps + 0.5f));
  if (rounded < 0) {
    rounded = 0;
  }
  if (rounded > sts::kMaxStep) {
    rounded = sts::kMaxStep;
  }
  return rounded;
}

float stepsToRad(const JointConfig& joint, int32_t steps) {
  return joint.direction * static_cast<float>(steps - joint.zero_steps) / kStepsPerRad;
}

// ---- controller ----

Controller::Controller(const Config& config, sts::Bus& bus, Output& output)
    : config_(config),
      bus_(bus),
      output_(output),
      config_error_(validateConfig(config)),
      state_(kIdle),
      last_set_ms_(0),
      last_feedback_ms_(0),
      last_torque_attempt_ms_(0),
      torque_unknown_(false),
      have_feedback_(false),
      last_config_error_ms_(0),
      config_error_reported_(false) {
  for (size_t j = 0; j < kNumJoints; ++j) {
    measured_rad_[j] = 0.0f;
    goal_rad_[j] = 0.0f;
    joint_failing_[j] = false;
    last_error_ms_[j] = 0;
  }
}

bool Controller::set(float pan, float tilt, uint32_t now_ms) {
  if (config_error_ != NULL) {
    char text[96];
    Text(text, sizeof(text)).add("invalid firmware config: ").add(config_error_);
    output_.error(text);
    return false;
  }
  if (!isFiniteValue(pan) || !isFiniteValue(tilt)) {
    output_.error("rejected non-finite SET");
    return false;
  }
  goal_rad_[kPan] = clampToLimits(config_.joints[kPan], pan);
  goal_rad_[kTilt] = clampToLimits(config_.joints[kTilt], tilt);
  if (state_ != kActive ||
      (torque_unknown_ &&
       static_cast<uint32_t>(now_ms - last_torque_attempt_ms_) >= config_.torque_retry_ms)) {
    enableTorque(now_ms);
  }
  uint8_t ids[kNumJoints];
  int32_t steps[kNumJoints];
  for (size_t j = 0; j < kNumJoints; ++j) {
    ids[j] = config_.joints[j].servo_id;
    steps[j] = radToSteps(config_.joints[j], goal_rad_[j]);
  }
  bus_.syncWritePosition(ids, steps, kNumJoints, config_.speed, config_.acceleration);
  state_ = kActive;
  last_set_ms_ = now_ms;
  return true;
}

void Controller::stop(uint32_t now_ms) {
  if (config_error_ != NULL) {
    return;
  }
  hold(now_ms);
  state_ = kHolding;
}

void Controller::update(uint32_t now_ms) {
  if (config_error_ != NULL) {
    if (!config_error_reported_ ||
        static_cast<uint32_t>(now_ms - last_config_error_ms_) >= config_.error_repeat_ms) {
      char text[96];
      Text(text, sizeof(text)).add("invalid firmware config: ").add(config_error_);
      output_.error(text);
      config_error_reported_ = true;
      last_config_error_ms_ = now_ms;
    }
    return;
  }
  if (state_ == kActive && static_cast<uint32_t>(now_ms - last_set_ms_) >= config_.watchdog_ms) {
    // No SET for watchdog_ms (host, link or tracker gone): stop where we are.
    hold(now_ms);
    state_ = kHolding;
  }
  if (static_cast<uint32_t>(now_ms - last_feedback_ms_) >= config_.feedback_period_ms) {
    last_feedback_ms_ = now_ms;
    readFeedback(now_ms);
  }
}

bool Controller::measured(float& pan, float& tilt) const {
  if (!have_feedback_) {
    return false;
  }
  pan = measured_rad_[kPan];
  tilt = measured_rad_[kTilt];
  return true;
}

bool Controller::readJoint(size_t joint, int32_t& steps, uint32_t now_ms) {
  const sts::Result result = bus_.readPosition(config_.joints[joint].servo_id, steps);
  if (result != sts::kOk) {
    torque_unknown_ = true;  // it may have lost power; re-enable on the next SET
    reportServo(joint, sts::resultName(result), now_ms, false);
    joint_failing_[joint] = true;
    return false;
  }
  const uint8_t status = bus_.lastStatus();
  if (status != 0) {
    char what[64];
    Text text(what, sizeof(what));
    text.add("fault ").addHex2(status);
    if (status & 0x01) text.add(" voltage");
    if (status & 0x02) text.add(" angle-sensor");
    if (status & 0x04) text.add(" overheat");
    if (status & 0x08) text.add(" overcurrent");
    if (status & 0x20) text.add(" overload");
    reportServo(joint, what, now_ms, false);
    joint_failing_[joint] = true;
  } else {
    joint_failing_[joint] = false;  // recovered: the next failure reports at once
  }
  measured_rad_[joint] = stepsToRad(config_.joints[joint], steps);
  return true;
}

void Controller::readFeedback(uint32_t now_ms) {
  int32_t steps[kNumJoints];
  bool ok = true;
  for (size_t j = 0; j < kNumJoints; ++j) {
    ok = readJoint(j, steps[j], now_ms) && ok;
  }
  have_feedback_ = ok;
  if (ok) {
    // Never report a joint that did not answer: the host treats POS as measured.
    output_.position(measured_rad_[kPan], measured_rad_[kTilt]);
  }
}

void Controller::enableTorque(uint32_t now_ms) {
  last_torque_attempt_ms_ = now_ms;
  bool ok = true;
  for (size_t j = 0; j < kNumJoints; ++j) {
    const sts::Result result = bus_.writeTorque(config_.joints[j].servo_id, true);
    if (result != sts::kOk) {
      char what[48];
      Text(what, sizeof(what)).add("torque enable ").add(sts::resultName(result));
      reportServo(j, what, now_ms, false);
      ok = false;
    }
  }
  torque_unknown_ = !ok;
}

void Controller::hold(uint32_t now_ms) {
  if (config_.stop_disables_torque) {
    for (size_t j = 0; j < kNumJoints; ++j) {
      const sts::Result result = bus_.writeTorque(config_.joints[j].servo_id, false);
      if (result != sts::kOk) {
        char what[48];
        Text(what, sizeof(what)).add("torque disable ").add(sts::resultName(result));
        reportServo(j, what, now_ms, true);
      }
    }
    return;
  }
  // Hold the measured position: a goal equal to the present position.
  uint8_t ids[kNumJoints];
  int32_t steps[kNumJoints];
  size_t count = 0;
  for (size_t j = 0; j < kNumJoints; ++j) {
    int32_t present = 0;
    if (readJoint(j, present, now_ms)) {
      ids[count] = config_.joints[j].servo_id;
      steps[count] = present;
      ++count;
    } else {
      reportServo(j, "cannot hold: no position reply", now_ms, true);
    }
  }
  if (count > 0) {
    bus_.syncWritePosition(ids, steps, count, config_.speed, config_.acceleration);
  }
}

void Controller::reportServo(size_t joint, const char* what, uint32_t now_ms, bool force) {
  if (!force && joint_failing_[joint] &&
      static_cast<uint32_t>(now_ms - last_error_ms_[joint]) < config_.error_repeat_ms) {
    return;
  }
  char text[96];
  Text(text, sizeof(text))
      .add("servo ")
      .addUint(config_.joints[joint].servo_id)
      .add(" (")
      .add(config_.joints[joint].name)
      .add(") ")
      .add(what);
  output_.error(text);
  last_error_ms_[joint] = now_ms;
}

// ---- line protocol ----

namespace {

bool parseNumber(const char* token, float& value) {
  char* end = NULL;
  const double parsed = strtod(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }
  value = static_cast<float>(parsed);
  return true;
}

Command invalid(const char* error) {
  Command command;
  command.type = Command::kInvalid;
  command.pan = 0.0f;
  command.tilt = 0.0f;
  command.error = error;
  return command;
}

}  // namespace

Command parseLine(const char* line) {
  char copy[LineReader::kCapacity + 1];
  const size_t len = strlen(line);
  if (len > LineReader::kCapacity) {
    return invalid("line too long");
  }
  memcpy(copy, line, len + 1);

  char* tokens[6];
  size_t count = 0;
  char* p = copy;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t' || *p == '\r') {
      *p++ = '\0';
    }
    if (*p == '\0') {
      break;
    }
    if (count == 6) {
      return invalid("too many fields");
    }
    tokens[count++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r') {
      ++p;
    }
  }

  Command command;
  command.type = Command::kNone;
  command.pan = 0.0f;
  command.tilt = 0.0f;
  command.error = NULL;
  if (count == 0) {
    return command;  // blank line
  }
  if (strcmp(tokens[0], "STOP") == 0) {
    if (count != 1) {
      return invalid("STOP takes no arguments");
    }
    command.type = Command::kStop;
    return command;
  }
  if (strcmp(tokens[0], "SET") != 0) {
    return invalid("unknown command");
  }
  if (count != 5) {
    return invalid("SET needs pan_joint <rad> tilt_joint <rad>");
  }
  bool have_pan = false;
  bool have_tilt = false;
  for (size_t i = 1; i < count; i += 2) {
    float value = 0.0f;
    if (!parseNumber(tokens[i + 1], value)) {
      return invalid("SET value is not a number");
    }
    if (!isFiniteValue(value)) {
      return invalid("rejected non-finite SET");
    }
    if (strcmp(tokens[i], "pan_joint") == 0 && !have_pan) {
      command.pan = value;
      have_pan = true;
    } else if (strcmp(tokens[i], "tilt_joint") == 0 && !have_tilt) {
      command.tilt = value;
      have_tilt = true;
    } else {
      return invalid("SET needs pan_joint <rad> tilt_joint <rad>");
    }
  }
  command.type = Command::kSet;
  return command;
}

size_t formatFixed(char* out, size_t cap, float value, int decimals) {
  if (cap == 0) {
    return 0;
  }
  Text text(out, cap);
  if (!isFiniteValue(value)) {
    return text.add("nan").length();
  }
  if (decimals < 0) {
    decimals = 0;
  }
  if (decimals > 9) {
    decimals = 9;
  }
  unsigned long long scale = 1;
  for (int i = 0; i < decimals; ++i) {
    scale *= 10;
  }
  double magnitude = value < 0 ? -static_cast<double>(value) : static_cast<double>(value);
  const double limit = 1.0e18;
  double scaled = magnitude * static_cast<double>(scale) + 0.5;
  if (scaled > limit) {
    scaled = limit;
  }
  const unsigned long long whole = static_cast<unsigned long long>(scaled);
  if (value < 0 && whole != 0) {
    text.add("-");
  }
  // Integer part (may exceed unsigned long on 32-bit targets only if huge)
  char digits[24];
  size_t n = 0;
  unsigned long long integer = whole / scale;
  do {
    digits[n++] = static_cast<char>('0' + static_cast<int>(integer % 10));
    integer /= 10;
  } while (integer != 0 && n < sizeof(digits));
  char integer_text[24];
  for (size_t i = 0; i < n; ++i) {
    integer_text[i] = digits[n - 1 - i];
  }
  integer_text[n] = '\0';
  text.add(integer_text);
  if (decimals > 0) {
    unsigned long long fraction = whole % scale;
    char fraction_text[12];
    for (int i = decimals - 1; i >= 0; --i) {
      fraction_text[i] = static_cast<char>('0' + static_cast<int>(fraction % 10));
      fraction /= 10;
    }
    fraction_text[decimals] = '\0';
    text.add(".").add(fraction_text);
  }
  return text.length();
}

size_t formatPosition(char* out, size_t cap, float pan, float tilt) {
  return Text(out, cap)
      .add("POS pan_joint ")
      .addFixed(pan, 6)
      .add(" tilt_joint ")
      .addFixed(tilt, 6)
      .add("\n")
      .length();
}

size_t formatError(char* out, size_t cap, const char* text) {
  Text line(out, cap);
  line.add("ERR ");
  char one[2] = {0, 0};
  for (const char* p = text; *p != '\0'; ++p) {
    one[0] = (*p == '\n' || *p == '\r') ? ' ' : *p;
    line.add(one);
  }
  return line.add("\n").length();
}

bool LineReader::feed(char c) {
  if (complete_) {
    length_ = 0;
    buffer_[0] = '\0';
    overflow_ = false;
    complete_ = false;
  }
  if (c == '\r') {
    return false;
  }
  if (c == '\n') {
    complete_ = true;
    overflow_ = discarding_;
    discarding_ = false;
    if (overflow_) {
      length_ = 0;
    }
    buffer_[length_] = '\0';
    return true;
  }
  if (discarding_) {
    return false;
  }
  if (length_ >= kCapacity) {
    discarding_ = true;
    return false;
  }
  buffer_[length_++] = c;
  buffer_[length_] = '\0';
  return false;
}

}  // namespace turret

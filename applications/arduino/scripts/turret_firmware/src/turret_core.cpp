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

// Servo shaft angle (rad) of a joint position: the joint angle itself, or a
// slide's rack travel over the pinion's pitch radius.
float shaftAngle(const JointConfig& joint, float value) {
  return joint.pinion_radius_m > 0.0f ? value / joint.pinion_radius_m : value;
}

const char* validateJoint(const JointConfig& joint) {
  if (joint.direction != 1 && joint.direction != -1) {
    return "direction must be +1 or -1";
  }
  if (joint.servo_id > 253) {
    return "servo id must be 0-253";
  }
  if (joint.zero_steps < 0 || joint.zero_steps > sts::kMaxStep) {
    return "zero_steps must be 0-4095";
  }
  if (!isFiniteValue(joint.lower) || !isFiniteValue(joint.upper) || !(joint.lower < joint.upper)) {
    return "joint limits must be finite with lower < upper";
  }
  if (!isFiniteValue(joint.pinion_radius_m) || joint.pinion_radius_m < 0.0f) {
    return "pinion_radius_m must be finite and >= 0";
  }
  if (joint.torque_limit > 1000) {
    return "torque_limit must be 0-1000";
  }
  // The limits must map inside one servo turn, or clamping to 0..4095
  // would silently move the joint somewhere else.
  const float a = joint.zero_steps + joint.direction * shaftAngle(joint, joint.lower) * kStepsPerRad;
  const float b = joint.zero_steps + joint.direction * shaftAngle(joint, joint.upper) * kStepsPerRad;
  if (a < -0.5f || a > sts::kMaxStep + 0.5f || b < -0.5f || b > sts::kMaxStep + 0.5f) {
    return "joint limits reach outside servo steps 0-4095";
  }
  return NULL;
}

}  // namespace

// ---- configuration and conversions ----

const char* validateConfig(const Config& config) {
  const size_t count = config.actuator_fitted ? kNumJoints : 2;
  for (size_t j = 0; j < count; ++j) {
    const char* error = validateJoint(config.joints[j]);
    if (error != NULL) {
      return error;
    }
  }
  if (config.joints[kPan].servo_id == config.joints[kTilt].servo_id) {
    return "pan and tilt servo ids must differ";
  }
  if (config.actuator_fitted) {
    for (size_t a = 0; a < kNumJoints; ++a) {
      for (size_t b = a + 1; b < kNumJoints; ++b) {
        if (config.joints[a].servo_id == config.joints[b].servo_id) {
          return "the launcher's servo ids must differ from each other and pan and tilt";
        }
      }
    }
    if (!(config.joints[kActuator].pinion_radius_m > 0.0f)) {
      return "the rack (a slide) needs pinion_radius_m > 0";
    }
  }
  if (config.watchdog_ms == 0 || config.feedback_period_ms == 0) {
    return "watchdog_ms and feedback_period_ms must be > 0";
  }
  return NULL;
}

float clampToLimits(const JointConfig& joint, float value) {
  if (value < joint.lower) {
    return joint.lower;
  }
  if (value > joint.upper) {
    return joint.upper;
  }
  return value;
}

float restPosition(const JointConfig& joint) { return joint.lower; }

int32_t toSteps(const JointConfig& joint, float value) {
  const float steps = joint.zero_steps + joint.direction * shaftAngle(joint, value) * kStepsPerRad;
  int32_t rounded = static_cast<int32_t>(std::floor(steps + 0.5f));
  if (rounded < 0) {
    rounded = 0;
  }
  if (rounded > sts::kMaxStep) {
    rounded = sts::kMaxStep;
  }
  return rounded;
}

float fromSteps(const JointConfig& joint, int32_t steps) {
  const float angle = joint.direction * static_cast<float>(steps - joint.zero_steps) / kStepsPerRad;
  return joint.pinion_radius_m > 0.0f ? angle * joint.pinion_radius_m : angle;
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
      have_launcher_(false),
      last_config_error_ms_(0),
      config_error_reported_(false),
      last_ignored_ms_(0),
      ignored_reported_(false) {
  for (size_t j = 0; j < kNumJoints; ++j) {
    measured_[j] = 0.0f;
    goal_[j] = 0.0f;
    joint_failing_[j] = false;
    last_error_ms_[j] = 0;
  }
  goal_[kActuator] = restPosition(config_.joints[kActuator]);
  goal_[kTrigger] = restPosition(config_.joints[kTrigger]);
}

bool Controller::set(float pan, float tilt, uint32_t now_ms) {
  return setJoints(pan, tilt, false, 0.0f, false, 0.0f, now_ms);
}

bool Controller::set(float pan, float tilt, float actuator, float trigger, uint32_t now_ms) {
  return setJoints(pan, tilt, true, actuator, true, trigger, now_ms);
}

bool Controller::setJoints(float pan, float tilt, bool has_actuator, float actuator,
                           bool has_trigger, float trigger, uint32_t now_ms) {
  if (config_error_ != NULL) {
    char text[96];
    Text(text, sizeof(text)).add("invalid firmware config: ").add(config_error_);
    output_.error(text);
    return false;
  }
  if (!isFiniteValue(pan) || !isFiniteValue(tilt) || (has_actuator && !isFiniteValue(actuator)) ||
      (has_trigger && !isFiniteValue(trigger))) {
    output_.error("rejected non-finite SET");
    return false;
  }
  if ((has_actuator || has_trigger) && !config_.actuator_fitted &&
      (!ignored_reported_ ||
       static_cast<uint32_t>(now_ms - last_ignored_ms_) >= config_.error_repeat_ms)) {
    output_.error("actuator_joint/trigger_joint ignored: no launcher fitted (turret_config.h)");
    ignored_reported_ = true;
    last_ignored_ms_ = now_ms;
  }
  goal_[kPan] = clampToLimits(config_.joints[kPan], pan);
  goal_[kTilt] = clampToLimits(config_.joints[kTilt], tilt);
  if (state_ != kActive ||
      (torque_unknown_ &&
       static_cast<uint32_t>(now_ms - last_torque_attempt_ms_) >= config_.torque_retry_ms)) {
    enableTorque(now_ms);
  }
  uint8_t ids[kNumJoints];
  int32_t steps[kNumJoints];
  size_t count = 0;
  for (size_t j = 0; j < 2; ++j) {
    ids[count] = config_.joints[j].servo_id;
    steps[count++] = toSteps(config_.joints[j], goal_[j]);
  }
  if (config_.actuator_fitted) {
    // The trigger leaves 0 only while SETs keep naming it; the rack moves
    // only while SETs keep naming it and is otherwise stopped where it is.
    const JointConfig& trigger_joint = config_.joints[kTrigger];
    goal_[kTrigger] = has_trigger ? clampToLimits(trigger_joint, trigger)
                                  : restPosition(trigger_joint);
    const JointConfig& rack = config_.joints[kActuator];
    int32_t present = 0;
    if (has_actuator) {
      goal_[kActuator] = clampToLimits(rack, actuator);
      ids[count] = rack.servo_id;
      steps[count++] = toSteps(rack, goal_[kActuator]);
    } else if (readJoint(kActuator, present, now_ms)) {
      goal_[kActuator] = measured_[kActuator];
      ids[count] = rack.servo_id;
      steps[count++] = present;
    }
    ids[count] = trigger_joint.servo_id;
    steps[count++] = toSteps(trigger_joint, goal_[kTrigger]);
  }
  bus_.syncWritePosition(ids, steps, count, config_.speed, config_.acceleration);
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
    // No SET for watchdog_ms (host, link or tracker gone): stop where we
    // are, the trigger back to 0.
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
  pan = measured_[kPan];
  tilt = measured_[kTilt];
  return true;
}

bool Controller::measuredLauncher(float& actuator, float& trigger) const {
  if (!have_launcher_) {
    return false;
  }
  actuator = measured_[kActuator];
  trigger = measured_[kTrigger];
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
  measured_[joint] = fromSteps(config_.joints[joint], steps);
  return true;
}

void Controller::readFeedback(uint32_t now_ms) {
  int32_t steps[kNumJoints];
  bool ok = true;
  for (size_t j = 0; j < 2; ++j) {
    ok = readJoint(j, steps[j], now_ms) && ok;
  }
  have_launcher_ = false;
  if (config_.actuator_fitted) {
    const bool rack = readJoint(kActuator, steps[kActuator], now_ms);
    const bool trigger = readJoint(kTrigger, steps[kTrigger], now_ms);
    have_launcher_ = rack && trigger;
  }
  have_feedback_ = ok;
  if (ok) {
    // Never report a joint that did not answer: the host treats POS as
    // measured. A silent launcher servo leaves pan and tilt reported without
    // the launcher.
    output_.position(measured_[kPan], measured_[kTilt], have_launcher_,
                     have_launcher_ ? measured_[kActuator] : 0.0f,
                     have_launcher_ ? measured_[kTrigger] : 0.0f);
  }
}

void Controller::enableTorque(uint32_t now_ms) {
  last_torque_attempt_ms_ = now_ms;
  bool ok = true;
  for (size_t j = 0; j < numJoints(); ++j) {
    const JointConfig& joint = config_.joints[j];
    sts::Result result = bus_.writeTorque(joint.servo_id, true);
    const char* what = "torque enable ";
    if (result == sts::kOk && joint.torque_limit > 0) {
      // RAM register: the servo forgets it when it loses power
      result = bus_.writeTorqueLimit(joint.servo_id, joint.torque_limit);
      what = "torque limit ";
    }
    if (result != sts::kOk) {
      char text[48];
      Text(text, sizeof(text)).add(what).add(sts::resultName(result));
      reportServo(j, text, now_ms, false);
      ok = false;
    }
  }
  torque_unknown_ = !ok;
}

void Controller::hold(uint32_t now_ms) {
  uint8_t ids[kNumJoints];
  int32_t steps[kNumJoints];
  size_t count = 0;
  for (size_t j = 0; j < 2; ++j) {
    if (config_.stop_disables_torque) {
      const sts::Result result = bus_.writeTorque(config_.joints[j].servo_id, false);
      if (result != sts::kOk) {
        char what[48];
        Text(what, sizeof(what)).add("torque disable ").add(sts::resultName(result));
        reportServo(j, what, now_ms, true);
      }
      continue;
    }
    // Hold the measured position: a goal equal to the present position.
    int32_t present = 0;
    if (readJoint(j, present, now_ms)) {
      ids[count] = config_.joints[j].servo_id;
      steps[count] = present;
      ++count;
    } else {
      reportServo(j, "cannot hold: no position reply", now_ms, true);
    }
  }
  if (config_.actuator_fitted) {
    // The launcher's safe state, whatever pan and tilt do: the rack stopped
    // where it is, the trigger at 0.
    int32_t present = 0;
    if (readJoint(kActuator, present, now_ms)) {
      goal_[kActuator] = measured_[kActuator];
      ids[count] = config_.joints[kActuator].servo_id;
      steps[count++] = present;
    } else {
      reportServo(kActuator, "cannot hold: no position reply", now_ms, true);
    }
    const JointConfig& trigger = config_.joints[kTrigger];
    goal_[kTrigger] = restPosition(trigger);
    ids[count] = trigger.servo_id;
    steps[count++] = toSteps(trigger, goal_[kTrigger]);
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

Command emptyCommand(Command::Type type) {
  Command command;
  command.type = type;
  command.pan = 0.0f;
  command.tilt = 0.0f;
  command.has_actuator = false;
  command.actuator = 0.0f;
  command.has_trigger = false;
  command.trigger = 0.0f;
  command.error = NULL;
  return command;
}

Command invalid(const char* error) {
  Command command = emptyCommand(Command::kInvalid);
  command.error = error;
  return command;
}

const char kSetUsage[] =
    "SET needs pan_joint <rad> tilt_joint <rad> [actuator_joint <m>] [trigger_joint <rad>]";

}  // namespace

Command parseLine(const char* line) {
  char copy[LineReader::kCapacity + 1];
  const size_t len = strlen(line);
  if (len > LineReader::kCapacity) {
    return invalid("line too long");
  }
  memcpy(copy, line, len + 1);

  char* tokens[10];
  size_t count = 0;
  char* p = copy;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t' || *p == '\r') {
      *p++ = '\0';
    }
    if (*p == '\0') {
      break;
    }
    if (count == 10) {
      return invalid("too many fields");
    }
    tokens[count++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r') {
      ++p;
    }
  }

  Command command = emptyCommand(Command::kNone);
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
  if (count != 5 && count != 7 && count != 9) {
    return invalid(kSetUsage);
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
    } else if (strcmp(tokens[i], "actuator_joint") == 0 && !command.has_actuator) {
      command.actuator = value;
      command.has_actuator = true;
    } else if (strcmp(tokens[i], "trigger_joint") == 0 && !command.has_trigger) {
      command.trigger = value;
      command.has_trigger = true;
    } else {
      return invalid(kSetUsage);
    }
  }
  if (!have_pan || !have_tilt) {
    return invalid(kSetUsage);
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
  return formatPosition(out, cap, pan, tilt, false, 0.0f, 0.0f);
}

size_t formatPosition(char* out, size_t cap, float pan, float tilt, bool has_launcher,
                      float actuator, float trigger) {
  Text line(out, cap);
  line.add("POS pan_joint ").addFixed(pan, 6).add(" tilt_joint ").addFixed(tilt, 6);
  if (has_launcher) {
    line.add(" actuator_joint ").addFixed(actuator, 6).add(" trigger_joint ").addFixed(trigger, 6);
  }
  return line.add("\n").length();
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

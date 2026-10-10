// Turret firmware core shared by the UNO R4 WiFi (USB serial line protocol)
// and UNO Q (RouterBridge RPC) front ends: joint limits, radian/step
// conversion, the command watchdog, STOP, servo error reporting, and the
// ASCII line protocol of applications/pantiltros/scripts/turret_arduino_bridge.py.
//
// No Arduino headers, so test/host_test.cpp compiles it on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "sts_servo.h"

namespace turret {

const size_t kNumJoints = 2;
const size_t kPan = 0;
const size_t kTilt = 1;

struct JointConfig {
  const char* name;   // ROS joint name: pan_joint / tilt_joint
  uint8_t servo_id;   // STS bus ID
  int32_t zero_steps; // servo position (0-4095) at 0 rad
  int8_t direction;   // +1: steps increase with the joint angle, -1: decrease
  float min_rad;      // joint limits enforced by the firmware
  float max_rad;
};

struct Config {
  JointConfig joints[kNumJoints];  // [kPan], [kTilt]
  uint16_t speed;                  // goal speed, steps/s (0 = servo maximum)
  uint8_t acceleration;            // goal acceleration, 100 steps/s^2 units
  uint32_t watchdog_ms;            // hold when no SET arrives for this long
  uint32_t feedback_period_ms;     // measured position report period
  uint32_t error_repeat_ms;        // minimum interval between repeated errors
  uint32_t torque_retry_ms;        // minimum interval between torque re-enables
  bool stop_disables_torque;       // STOP/watchdog: torque off instead of holding
};

// Returns NULL if the configuration is usable, else the reason.
const char* validateConfig(const Config& config);

float clampToLimits(const JointConfig& joint, float rad);
// Joint angle to servo steps, clamped to 0..4095 (not to the joint limits).
int32_t radToSteps(const JointConfig& joint, float rad);
float stepsToRad(const JointConfig& joint, int32_t steps);

// Where the controller reports measured positions and errors.
class Output {
 public:
  virtual ~Output() {}
  virtual void position(float pan, float tilt) = 0;
  virtual void error(const char* text) = 0;
};

class Controller {
 public:
  enum State {
    kIdle,     // no SET since power-up: servos untouched
    kActive,   // following SET commands
    kHolding,  // stopped by STOP or the watchdog until the next SET
  };

  Controller(const Config& config, sts::Bus& bus, Output& output);

  // Clamp to the joint limits and command the servos. False (and an error
  // report) for non-finite values or an invalid configuration.
  bool set(float pan, float tilt, uint32_t now_ms);
  // Stop moving: hold the measured position (or disable torque).
  void stop(uint32_t now_ms);
  // Run the watchdog and the periodic position feedback.
  void update(uint32_t now_ms);

  // Last measured position (radians), if both servos answered at the last read.
  bool measured(float& pan, float& tilt) const;
  State state() const { return state_; }
  // Last commanded goal, after clamping (radians)
  float goal(size_t joint) const { return goal_rad_[joint]; }

 private:
  bool readJoint(size_t joint, int32_t& steps, uint32_t now_ms);
  void readFeedback(uint32_t now_ms);
  void enableTorque(uint32_t now_ms);
  void hold(uint32_t now_ms);
  void reportServo(size_t joint, const char* what, uint32_t now_ms, bool force);

  const Config& config_;
  sts::Bus& bus_;
  Output& output_;
  const char* config_error_;
  State state_;
  uint32_t last_set_ms_;
  uint32_t last_feedback_ms_;
  uint32_t last_torque_attempt_ms_;
  bool torque_unknown_;  // a servo stopped answering: re-enable torque on the next SET
  bool have_feedback_;
  float measured_rad_[kNumJoints];
  float goal_rad_[kNumJoints];
  bool joint_failing_[kNumJoints];
  uint32_t last_error_ms_[kNumJoints];
  uint32_t last_config_error_ms_;
  bool config_error_reported_;
};

// ---- ASCII line protocol (UNO R4 WiFi USB serial, 115200 baud) ----
//   SET pan_joint <rad> tilt_joint <rad>   host -> firmware
//   STOP                                   host -> firmware
//   POS pan_joint <rad> tilt_joint <rad>   firmware -> host
//   ERR <text>                             firmware -> host

struct Command {
  enum Type { kNone, kSet, kStop, kInvalid };
  Type type;
  float pan;
  float tilt;
  const char* error;  // set for kInvalid
};

Command parseLine(const char* line);

// Fixed-point formatting without printf (newlib-nano has no %f).
size_t formatFixed(char* out, size_t cap, float value, int decimals);
size_t formatPosition(char* out, size_t cap, float pan, float tilt);
size_t formatError(char* out, size_t cap, const char* text);

// Collects characters into lines; '\r' is ignored, over-long lines are dropped.
class LineReader {
 public:
  static const size_t kCapacity = 96;
  LineReader() : length_(0), overflow_(false), discarding_(false), complete_(false) {
    buffer_[0] = '\0';
  }
  // True when `c` completes a line: line() is then valid until the next feed().
  bool feed(char c);
  const char* line() const { return buffer_; }
  // True if the line just completed was too long and has been discarded.
  bool overflowed() const { return overflow_; }

 private:
  char buffer_[kCapacity + 1];
  size_t length_;
  bool overflow_;
  bool discarding_;
  bool complete_;
};

}  // namespace turret

// Turret firmware core shared by the UNO R4 WiFi (USB serial line protocol)
// and UNO Q (RouterBridge RPC) front ends: joint limits, joint position/step
// conversion, the command watchdog, STOP, servo error reporting, and the
// ASCII line protocol of applications/pantiltros/scripts/turret_arduino_bridge.py.
//
// Joints: pan and tilt (revolute, radians) and, when fitted, the foam dart
// launcher (systems/racecar_turret/interfaces.md, "Foam dart launcher"): the
// priming rack (actuator_joint, a rack-and-pinion slide, metres, 0 forward)
// and the trigger (trigger_joint, radians, 0 safe). Its safe state is the
// trigger at 0 and the rack stopped: a SET that does not name the trigger
// returns it to 0, one that does not name the rack stops it where it is, and
// STOP and the watchdog do both, while pan and tilt keep their hold
// behaviour.
//
// No Arduino headers, so test/host_test.cpp compiles it on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "sts_servo.h"

namespace turret {

const size_t kNumJoints = 4;
const size_t kPan = 0;
const size_t kTilt = 1;
const size_t kActuator = 2;  // the launcher's priming rack, when fitted
const size_t kTrigger = 3;   // the launcher's trigger, when fitted

struct JointConfig {
  const char* name;       // ROS joint name: pan_joint, tilt_joint, actuator_joint, trigger_joint
  uint8_t servo_id;       // STS bus ID
  int32_t zero_steps;     // servo position (0-4095) at joint position 0
  int8_t direction;       // +1: steps increase with the joint position, -1: decrease
  float lower;            // joint limits enforced by the firmware (rad, or m for a slide);
  float upper;            // the lower one is the launcher joints' rest (safe) position
  float pinion_radius_m;  // 0: revolute joint, positions in radians; > 0: a
                          // rack-and-pinion slide, positions in metres of rack
                          // travel (the pinion's pitch radius)
  uint16_t torque_limit;  // 0: the servo's own; 1-1000 (0.1 %): written at every torque enable
};

struct Config {
  JointConfig joints[kNumJoints];  // [kPan], [kTilt], [kActuator], [kTrigger]
  uint16_t speed;                  // goal speed, steps/s (0 = servo maximum)
  uint8_t acceleration;            // goal acceleration, 100 steps/s^2 units
  uint32_t watchdog_ms;            // hold when no SET arrives for this long
  uint32_t feedback_period_ms;     // measured position report period
  uint32_t error_repeat_ms;        // minimum interval between repeated errors
  uint32_t torque_retry_ms;        // minimum interval between torque re-enables
  bool stop_disables_torque;       // STOP/watchdog: pan/tilt torque off instead of holding
  bool actuator_fitted;            // the dart launcher: drive joints[kActuator] and [kTrigger]
};

// Returns NULL if the configuration is usable, else the reason.
const char* validateConfig(const Config& config);

float clampToLimits(const JointConfig& joint, float value);
// A launcher joint's rest (safe) position: its lower limit (rack forward, trigger safe).
float restPosition(const JointConfig& joint);
// Joint position to servo steps, clamped to 0..4095 (not to the joint limits).
int32_t toSteps(const JointConfig& joint, float value);
float fromSteps(const JointConfig& joint, int32_t steps);

// Where the controller reports measured positions and errors.
class Output {
 public:
  virtual ~Output() {}
  // has_launcher: the launcher is fitted and both its servos answered (else
  // actuator and trigger are 0).
  virtual void position(float pan, float tilt, bool has_launcher, float actuator,
                        float trigger) = 0;
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

  // Clamp to the joint limits and command the servos; a fitted launcher's
  // trigger goes to 0 and its rack stops where it is. False (and an error
  // report) for non-finite values or an invalid configuration.
  bool set(float pan, float tilt, uint32_t now_ms);
  // The same with the launcher's goals (rack metres, trigger radians), which
  // are ignored, with a rate-limited error, when no launcher is fitted.
  bool set(float pan, float tilt, float actuator, float trigger, uint32_t now_ms);
  // Either launcher goal may be missing: the trigger then goes to 0, the
  // rack stops where it is (the line protocol's optional fields).
  bool setJoints(float pan, float tilt, bool has_actuator, float actuator, bool has_trigger,
                 float trigger, uint32_t now_ms);
  // Stop moving: hold the measured pan/tilt position (or disable their
  // torque), return the trigger to 0 and hold the rack where it is.
  void stop(uint32_t now_ms);
  // Run the watchdog and the periodic position feedback.
  void update(uint32_t now_ms);

  // Last measured pan/tilt position (radians), if both answered at the last read.
  bool measured(float& pan, float& tilt) const;
  // Last measured launcher position (rack metres, trigger radians), if it is
  // fitted and both its servos answered.
  bool measuredLauncher(float& actuator, float& trigger) const;
  size_t numJoints() const { return config_.actuator_fitted ? kNumJoints : 2; }
  State state() const { return state_; }
  // Last commanded goal, after clamping (radians, metres for the rack)
  float goal(size_t joint) const { return goal_[joint]; }

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
  bool have_launcher_;  // both launcher servos answered at the last read
  float measured_[kNumJoints];
  float goal_[kNumJoints];
  bool joint_failing_[kNumJoints];
  uint32_t last_error_ms_[kNumJoints];
  uint32_t last_config_error_ms_;
  bool config_error_reported_;
  uint32_t last_ignored_ms_;  // last "no launcher fitted" report
  bool ignored_reported_;
};

// ---- ASCII line protocol (UNO R4 WiFi USB serial, 115200 baud) ----
//   SET pan_joint <rad> tilt_joint <rad> [actuator_joint <m>] [trigger_joint <rad>]
//                                                          host -> firmware
//   STOP                                                   host -> firmware
//   POS pan_joint <rad> tilt_joint <rad> [actuator_joint <m> trigger_joint <rad>]
//                                                          firmware -> host
//   ERR <text>                                             firmware -> host
// A SET without trigger_joint returns a fitted trigger to 0; one without
// actuator_joint stops the rack where it is. POS carries the launcher when
// it is fitted and both its servos answered.

struct Command {
  enum Type { kNone, kSet, kStop, kInvalid };
  Type type;
  float pan;
  float tilt;
  bool has_actuator;  // kSet named actuator_joint
  float actuator;
  bool has_trigger;   // kSet named trigger_joint
  float trigger;
  const char* error;  // set for kInvalid
};

Command parseLine(const char* line);

// Fixed-point formatting without printf (newlib-nano has no %f).
size_t formatFixed(char* out, size_t cap, float value, int decimals);
size_t formatPosition(char* out, size_t cap, float pan, float tilt);
size_t formatPosition(char* out, size_t cap, float pan, float tilt, bool has_launcher,
                      float actuator, float trigger);
size_t formatError(char* out, size_t cap, const char* text);

// Collects characters into lines; '\r' is ignored, over-long lines are dropped.
class LineReader {
 public:
  static const size_t kCapacity = 128;
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

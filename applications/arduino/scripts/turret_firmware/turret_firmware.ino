// PanTiltROS turret firmware: two Waveshare ST3215 / Feetech STS3215 serial
// bus servos (pan and tilt) on a half-duplex adapter at the board's D0/D1
// UART (Serial1), commanded by applications/pantiltros/scripts/turret_arduino_bridge.py.
//
// One sketch, two front ends over the same core (src/turret_core.*):
//   Arduino UNO R4 WiFi (arduino:renesas_uno:unor4wifi): the bridge's ASCII
//     line protocol on the USB serial port, 115200 baud
//       SET pan_joint <rad> tilt_joint <rad> / STOP      host -> firmware
//       POS pan_joint <rad> tilt_joint <rad> / ERR <text> firmware -> host
//   Arduino UNO Q MCU (arduino:zephyr:unoq): the same messages as RouterBridge
//     RPC through arduino-router to the board's Linux side
//       turret/set(pan, tilt) -> bool    turret/stop() -> bool
//       turret/get_position() -> [pan, tilt], or [] without a fresh reading
//       notifications turret/pos [pan, tilt] and turret/err [text]
//
// Both: joint limits, a 500 ms watchdog that holds position when SETs stop,
// STOP holds the measured position, POS at 25 Hz only from servo replies, and
// ERR when a servo does not answer. Settings are in src/turret_config.h.

#include "src/turret_config.h"

#if defined(ARDUINO_UNO_Q)
#define TURRET_UNO_Q 1
#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <cmath>
#elif defined(ARDUINO_ARCH_RENESAS_UNO)
#define TURRET_UNO_Q 0
#else
#error "turret_firmware targets the UNO R4 WiFi (arduino:renesas_uno:unor4wifi) and the UNO Q (arduino:zephyr:unoq)"
#endif

// Half-duplex servo bus on Serial1 (D0 RX, D1 TX)
class SerialByteLink : public turret::sts::ByteLink {
 public:
  SerialByteLink(HardwareSerial& port, int dir_pin) : port_(port), dir_pin_(dir_pin) {}

  void begin(unsigned long baud) {
    if (dir_pin_ >= 0) {
      pinMode(dir_pin_, OUTPUT);
      digitalWrite(dir_pin_, LOW);
    }
    port_.begin(baud);
  }

  void write(const uint8_t* data, size_t len) override {
    if (dir_pin_ >= 0) {
      digitalWrite(dir_pin_, HIGH);
    }
    port_.write(data, len);
    port_.flush();  // returns once the last stop bit is out
    if (dir_pin_ >= 0) {
      digitalWrite(dir_pin_, LOW);
    }
  }

  int read() override { return port_.available() > 0 ? port_.read() : -1; }

  uint32_t micros() override { return ::micros(); }

 private:
  HardwareSerial& port_;
  int dir_pin_;
};

SerialByteLink servo_link(Serial1, TURRET_SERVO_DIR_PIN);
turret::sts::Bus servo_bus(servo_link, TURRET_SERVO_REPLY_TIMEOUT_US);

#if TURRET_UNO_Q
// ---------------------------------------------------------------------------
// UNO Q: RouterBridge RPC. Bridge.provide() handlers run in the Bridge's RPC
// thread (small stack, higher priority than loop()), so they only exchange
// values with loop() through this mutex-protected mailbox; all servo traffic
// happens in loop().

struct Mailbox {
  bool set_pending;
  float set_pan;
  float set_tilt;
  bool stop_pending;
  bool have_position;
  float pan;
  float tilt;
};

Mailbox mailbox = {false, 0.0f, 0.0f, false, false, 0.0f, 0.0f};
struct k_mutex mailbox_lock;
bool rpc_registered = false;
uint32_t last_registration_error_ms = 0;

bool rpcSet(float pan, float tilt) {
  if (!std::isfinite(pan) || !std::isfinite(tilt)) {
    return false;
  }
  k_mutex_lock(&mailbox_lock, K_FOREVER);
  mailbox.set_pending = true;
  mailbox.set_pan = pan;
  mailbox.set_tilt = tilt;
  k_mutex_unlock(&mailbox_lock);
  return true;
}

bool rpcStop() {
  k_mutex_lock(&mailbox_lock, K_FOREVER);
  mailbox.stop_pending = true;
  mailbox.set_pending = false;  // a STOP cancels an earlier, unapplied SET
  k_mutex_unlock(&mailbox_lock);
  return true;
}

MsgPack::arr_t<float> rpcGetPosition() {
  MsgPack::arr_t<float> position;
  k_mutex_lock(&mailbox_lock, K_FOREVER);
  if (mailbox.have_position) {
    position.push_back(mailbox.pan);
    position.push_back(mailbox.tilt);
  }
  k_mutex_unlock(&mailbox_lock);
  return position;
}

class BridgeOutput : public turret::Output {
 public:
  void position(float pan, float tilt) override { Bridge.notify("turret/pos", pan, tilt); }
  void error(const char* text) override { Bridge.notify("turret/err", MsgPack::str_t(text)); }
};

BridgeOutput output;
turret::Controller controller(kTurretConfig, servo_bus, output);

void setup() {
  k_mutex_init(&mailbox_lock);
  Bridge.begin();
  servo_link.begin(TURRET_SERVO_BAUD);
  rpc_registered = Bridge.provide("turret/set", rpcSet);
  rpc_registered = Bridge.provide("turret/stop", rpcStop) && rpc_registered;
  rpc_registered = Bridge.provide("turret/get_position", rpcGetPosition) && rpc_registered;
}

void loop() {
  k_mutex_lock(&mailbox_lock, K_FOREVER);
  const bool do_stop = mailbox.stop_pending;
  const bool do_set = mailbox.set_pending;
  const float pan = mailbox.set_pan;
  const float tilt = mailbox.set_tilt;
  mailbox.stop_pending = false;
  mailbox.set_pending = false;
  k_mutex_unlock(&mailbox_lock);

  if (do_stop) {
    controller.stop(millis());
  }
  if (do_set) {  // after the STOP: a SET that arrived later releases it
    controller.set(pan, tilt, millis());
  }
  controller.update(millis());

  float measured_pan = 0.0f;
  float measured_tilt = 0.0f;
  const bool have_position = controller.measured(measured_pan, measured_tilt);
  k_mutex_lock(&mailbox_lock, K_FOREVER);
  mailbox.have_position = have_position;
  mailbox.pan = measured_pan;
  mailbox.tilt = measured_tilt;
  k_mutex_unlock(&mailbox_lock);

  if (!rpc_registered && millis() - last_registration_error_ms >= 2000) {
    last_registration_error_ms = millis();
    output.error("could not register the turret/* RPC methods (another client owns them?)");
  }
  delay(1);
}

#else
// ---------------------------------------------------------------------------
// UNO R4 WiFi: ASCII lines on the USB serial port.

class SerialOutput : public turret::Output {
 public:
  void position(float pan, float tilt) override {
    char line[64];
    const size_t len = turret::formatPosition(line, sizeof(line), pan, tilt);
    Serial.write(reinterpret_cast<const uint8_t*>(line), len);
  }
  void error(const char* text) override {
    char line[128];
    const size_t len = turret::formatError(line, sizeof(line), text);
    Serial.write(reinterpret_cast<const uint8_t*>(line), len);
  }
};

SerialOutput output;
turret::Controller controller(kTurretConfig, servo_bus, output);
turret::LineReader reader;

void handleLine() {
  if (reader.overflowed()) {
    output.error("line too long");
    return;
  }
  const turret::Command command = turret::parseLine(reader.line());
  switch (command.type) {
    case turret::Command::kSet:
      controller.set(command.pan, command.tilt, millis());
      break;
    case turret::Command::kStop:
      controller.stop(millis());
      break;
    case turret::Command::kInvalid:
      output.error(command.error);
      break;
    case turret::Command::kNone:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  servo_link.begin(TURRET_SERVO_BAUD);
}

void loop() {
  // Bounded, so a flood of input cannot starve the watchdog and feedback
  for (int i = 0; i < 128 && Serial.available() > 0; ++i) {
    if (reader.feed(static_cast<char>(Serial.read()))) {
      handleLine();
    }
  }
  controller.update(millis());
}

#endif

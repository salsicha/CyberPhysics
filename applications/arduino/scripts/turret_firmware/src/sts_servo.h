// Minimal Feetech STS (SMS/STS series, e.g. Waveshare ST3215 / Feetech
// STS3215) serial-bus servo protocol.
//
// Instruction packet: 0xFF 0xFF ID LEN INSTR PARAM... CHECKSUM
// Status packet:      0xFF 0xFF ID LEN ERROR PARAM... CHECKSUM
// LEN counts INSTR/ERROR, the parameters and the checksum (parameters + 2);
// CHECKSUM = ~(ID + LEN + INSTR/ERROR + parameters), low byte. STS servos
// store 16-bit registers little endian (low byte first).
//
// No Arduino headers: the same code is compiled for the UNO R4 WiFi, the
// UNO Q MCU and the host unit tests (test/host_test.cpp).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace turret {
namespace sts {

const uint8_t kBroadcastId = 0xFE;

const uint8_t kInstPing = 0x01;
const uint8_t kInstRead = 0x02;
const uint8_t kInstWrite = 0x03;
const uint8_t kInstSyncWrite = 0x83;

// Memory table (SMS_STS.h in Feetech's SCServo library)
const uint8_t kRegTorqueEnable = 40;  // 0 off, 1 on; 128 recalibrates the midpoint: never written here
const uint8_t kRegAcceleration = 41;  // 41 acc, 42-43 goal position, 44-45 goal time, 46-47 goal speed
const uint8_t kRegGoalPosition = 42;
const uint8_t kRegTorqueLimit = 48;  // 48-49, 0.1 % units; reset from EEPROM's max torque at power-up
const uint8_t kRegPresentPosition = 56;

const int32_t kStepsPerTurn = 4096;
const int32_t kMaxStep = kStepsPerTurn - 1;

// Largest packet built or parsed here (a four-servo sync write is 40 bytes)
const size_t kMaxPacket = 64;
const size_t kMaxStatusParams = kMaxPacket - 6;

// ~(sum of bytes) over ID..last parameter
uint8_t checksum(const uint8_t* body, size_t len);

// Each builder returns the packet length, or 0 if it does not fit in `cap`.
size_t buildPacket(uint8_t* out, size_t cap, uint8_t id, uint8_t instruction,
                   const uint8_t* params, size_t num_params);
size_t buildPing(uint8_t* out, size_t cap, uint8_t id);
size_t buildRead(uint8_t* out, size_t cap, uint8_t id, uint8_t address, uint8_t count);
size_t buildWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t address,
                  const uint8_t* data, size_t len);
// SYNC WRITE (broadcast, no reply) of acceleration, goal position, goal time 0
// and goal speed (registers 41-47) for `count` servos, as SMS_STS::SyncWritePosEx.
size_t buildSyncWritePosition(uint8_t* out, size_t cap, const uint8_t* ids,
                              const int32_t* steps, size_t count, uint16_t speed,
                              uint8_t acceleration);

// Position registers use bit 15 as the sign (sign and magnitude).
uint16_t encodePosition(int32_t steps);
int32_t decodePosition(uint16_t raw);

// Incremental parser for status packets. Leading garbage and extra 0xFF
// header bytes are skipped; packets with a bad length or checksum are dropped
// and counted.
class StatusParser {
 public:
  StatusParser() : bad_packets_(0) { reset(); }
  void reset();
  // Returns true when `byte` completes a valid packet.
  bool feed(uint8_t byte);

  uint8_t id() const { return id_; }
  uint8_t error() const { return error_; }
  const uint8_t* params() const { return params_; }
  size_t numParams() const { return num_params_; }
  // True if the completed packet is byte for byte `packet` (a half-duplex
  // echo of what was just sent).
  bool equals(const uint8_t* packet, size_t len) const;
  uint32_t badPackets() const { return bad_packets_; }

 private:
  enum State { kHeader1, kHeader2, kId, kLength, kBody };
  State state_;
  uint8_t id_;
  uint8_t length_;
  uint8_t error_;
  uint8_t params_[kMaxStatusParams];
  size_t num_params_;
  size_t body_received_;
  uint8_t sum_;
  uint32_t bad_packets_;
};

// Byte transport to the half-duplex bus. write() must return only after the
// last bit has left the UART (and the direction pin, if any, is back to
// receive), so the reply is not missed.
class ByteLink {
 public:
  virtual ~ByteLink() {}
  virtual void write(const uint8_t* data, size_t len) = 0;
  virtual int read() = 0;  // next received byte, or -1
  virtual uint32_t micros() = 0;
  virtual void idle() {}  // called while waiting for a reply
};

enum Result { kOk = 0, kTimeout, kBadReply, kTooLong };

const char* resultName(Result result);

class Bus {
 public:
  Bus(ByteLink& link, uint32_t reply_timeout_us)
      : link_(link), reply_timeout_us_(reply_timeout_us), last_status_(0) {}

  Result ping(uint8_t id);
  Result readPosition(uint8_t id, int32_t& steps);
  Result writeTorque(uint8_t id, bool enable);
  // Torque limit (0-1000, 0.1 % of the maximum; RAM, until the next power-up)
  Result writeTorqueLimit(uint8_t id, uint16_t limit);
  Result syncWritePosition(const uint8_t* ids, const int32_t* steps, size_t count,
                           uint16_t speed, uint8_t acceleration);
  // Error byte of the last status packet (0 = no fault)
  uint8_t lastStatus() const { return last_status_; }

 private:
  Result transact(const uint8_t* packet, size_t len, uint8_t id, StatusParser& reply);
  void drainInput();

  ByteLink& link_;
  uint32_t reply_timeout_us_;
  uint8_t last_status_;
};

}  // namespace sts
}  // namespace turret

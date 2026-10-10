#include "sts_servo.h"

#include <string.h>

namespace turret {
namespace sts {

uint8_t checksum(const uint8_t* body, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + body[i]);
  }
  return static_cast<uint8_t>(~sum);
}

size_t buildPacket(uint8_t* out, size_t cap, uint8_t id, uint8_t instruction,
                   const uint8_t* params, size_t num_params) {
  const size_t total = num_params + 6;
  if (total > cap || num_params + 2 > 0xFF) {
    return 0;
  }
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = id;
  out[3] = static_cast<uint8_t>(num_params + 2);
  out[4] = instruction;
  if (num_params > 0) {
    memcpy(out + 5, params, num_params);
  }
  out[5 + num_params] = checksum(out + 2, num_params + 3);
  return total;
}

size_t buildPing(uint8_t* out, size_t cap, uint8_t id) {
  return buildPacket(out, cap, id, kInstPing, NULL, 0);
}

size_t buildRead(uint8_t* out, size_t cap, uint8_t id, uint8_t address, uint8_t count) {
  const uint8_t params[2] = {address, count};
  return buildPacket(out, cap, id, kInstRead, params, 2);
}

size_t buildWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t address,
                  const uint8_t* data, size_t len) {
  uint8_t params[kMaxPacket];
  if (len + 1 > sizeof(params)) {
    return 0;
  }
  params[0] = address;
  memcpy(params + 1, data, len);
  return buildPacket(out, cap, id, kInstWrite, params, len + 1);
}

size_t buildSyncWritePosition(uint8_t* out, size_t cap, const uint8_t* ids,
                              const int32_t* steps, size_t count, uint16_t speed,
                              uint8_t acceleration) {
  const size_t per_servo = 7;  // acc, position L/H, time L/H, speed L/H
  uint8_t params[kMaxPacket];
  const size_t num_params = 2 + count * (per_servo + 1);
  if (count == 0 || num_params > sizeof(params)) {
    return 0;
  }
  params[0] = kRegAcceleration;
  params[1] = static_cast<uint8_t>(per_servo);
  size_t at = 2;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t position = encodePosition(steps[i]);
    params[at++] = ids[i];
    params[at++] = acceleration;
    params[at++] = static_cast<uint8_t>(position & 0xFF);
    params[at++] = static_cast<uint8_t>(position >> 8);
    params[at++] = 0;  // goal time: unused in position mode
    params[at++] = 0;
    params[at++] = static_cast<uint8_t>(speed & 0xFF);
    params[at++] = static_cast<uint8_t>(speed >> 8);
  }
  return buildPacket(out, cap, kBroadcastId, kInstSyncWrite, params, num_params);
}

uint16_t encodePosition(int32_t steps) {
  if (steps < 0) {
    int32_t magnitude = -steps;
    if (magnitude > 0x7FFF) {
      magnitude = 0x7FFF;
    }
    return static_cast<uint16_t>(magnitude | 0x8000);
  }
  if (steps > 0x7FFF) {
    steps = 0x7FFF;
  }
  return static_cast<uint16_t>(steps);
}

int32_t decodePosition(uint16_t raw) {
  if (raw & 0x8000) {
    return -static_cast<int32_t>(raw & 0x7FFF);
  }
  return static_cast<int32_t>(raw);
}

void StatusParser::reset() {
  state_ = kHeader1;
  id_ = 0;
  length_ = 0;
  error_ = 0;
  num_params_ = 0;
  body_received_ = 0;
  sum_ = 0;
}

bool StatusParser::feed(uint8_t byte) {
  switch (state_) {
    case kHeader1:
      if (byte == 0xFF) {
        state_ = kHeader2;
      }
      return false;
    case kHeader2:
      state_ = byte == 0xFF ? kId : kHeader1;
      return false;
    case kId:
      if (byte == 0xFF) {
        return false;  // extra header byte; IDs are 0-253
      }
      id_ = byte;
      sum_ = byte;
      state_ = kLength;
      return false;
    case kLength:
      if (byte < 2 || byte > kMaxStatusParams + 2) {
        ++bad_packets_;
        state_ = byte == 0xFF ? kHeader2 : kHeader1;
        return false;
      }
      length_ = byte;
      sum_ = static_cast<uint8_t>(sum_ + byte);
      body_received_ = 0;
      num_params_ = 0;
      state_ = kBody;
      return false;
    case kBody:
      ++body_received_;
      if (body_received_ == length_) {  // checksum byte
        state_ = kHeader1;
        if (static_cast<uint8_t>(~sum_) != byte) {
          ++bad_packets_;
          return false;
        }
        return true;
      }
      sum_ = static_cast<uint8_t>(sum_ + byte);
      if (body_received_ == 1) {
        error_ = byte;
      } else {
        params_[num_params_++] = byte;
      }
      return false;
  }
  return false;
}

bool StatusParser::equals(const uint8_t* packet, size_t len) const {
  if (len != num_params_ + 6 || packet[2] != id_ || packet[3] != length_ ||
      packet[4] != error_) {
    return false;
  }
  return num_params_ == 0 || memcmp(packet + 5, params_, num_params_) == 0;
}

const char* resultName(Result result) {
  switch (result) {
    case kOk:
      return "ok";
    case kTimeout:
      return "timeout";
    case kBadReply:
      return "bad reply";
    case kTooLong:
      return "packet too long";
  }
  return "?";
}

void Bus::drainInput() {
  while (link_.read() >= 0) {
  }
}

Result Bus::transact(const uint8_t* packet, size_t len, uint8_t id, StatusParser& reply) {
  if (len == 0) {
    return kTooLong;
  }
  drainInput();
  link_.write(packet, len);
  reply.reset();
  const uint32_t bad_before = reply.badPackets();
  const uint32_t start = link_.micros();
  while (static_cast<uint32_t>(link_.micros() - start) < reply_timeout_us_) {
    const int byte = link_.read();
    if (byte < 0) {
      link_.idle();
      continue;
    }
    if (!reply.feed(static_cast<uint8_t>(byte))) {
      continue;
    }
    if (reply.equals(packet, len)) {
      continue;  // our own packet echoed by a one-wire adapter
    }
    if (reply.id() == id) {
      last_status_ = reply.error();
      return kOk;
    }
  }
  return reply.badPackets() != bad_before ? kBadReply : kTimeout;
}

Result Bus::ping(uint8_t id) {
  uint8_t packet[kMaxPacket];
  StatusParser reply;
  return transact(packet, buildPing(packet, sizeof(packet), id), id, reply);
}

Result Bus::readPosition(uint8_t id, int32_t& steps) {
  uint8_t packet[kMaxPacket];
  StatusParser reply;
  const size_t len = buildRead(packet, sizeof(packet), id, kRegPresentPosition, 2);
  const Result result = transact(packet, len, id, reply);
  if (result != kOk) {
    return result;
  }
  if (reply.numParams() != 2) {
    return kBadReply;
  }
  const uint16_t raw =
      static_cast<uint16_t>(reply.params()[0] | (static_cast<uint16_t>(reply.params()[1]) << 8));
  steps = decodePosition(raw);
  return kOk;
}

Result Bus::writeTorque(uint8_t id, bool enable) {
  uint8_t packet[kMaxPacket];
  StatusParser reply;
  const uint8_t value = enable ? 1 : 0;
  const size_t len = buildWrite(packet, sizeof(packet), id, kRegTorqueEnable, &value, 1);
  return transact(packet, len, id, reply);
}

Result Bus::syncWritePosition(const uint8_t* ids, const int32_t* steps, size_t count,
                              uint16_t speed, uint8_t acceleration) {
  uint8_t packet[kMaxPacket];
  const size_t len =
      buildSyncWritePosition(packet, sizeof(packet), ids, steps, count, speed, acceleration);
  if (len == 0) {
    return kTooLong;
  }
  drainInput();
  link_.write(packet, len);  // broadcast: the servos do not reply
  return kOk;
}

}  // namespace sts
}  // namespace turret

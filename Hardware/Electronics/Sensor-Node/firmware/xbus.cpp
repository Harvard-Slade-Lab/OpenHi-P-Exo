#include "xbus.h"

#include <string.h>

namespace xbus {

size_t build(uint8_t mid, const uint8_t *data, uint8_t len, uint8_t *out, size_t cap) {
  if (len > 254 || cap < static_cast<size_t>(len) + 5) return 0;
  out[0] = kPreamble;
  out[1] = kBusId;
  out[2] = mid;
  out[3] = len;
  uint8_t sum = static_cast<uint8_t>(kBusId + mid + len);
  for (uint8_t i = 0; i < len; i++) {
    out[4 + i] = data[i];
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  out[4 + len] = static_cast<uint8_t>(-sum);
  return static_cast<size_t>(len) + 5;
}

bool Parser::feed(uint8_t b, Message &msg) {
  switch (state_) {
    case kPre:
      if (b == kPreamble) state_ = kBid;
      return false;
    case kBid:
      if (b == kBusId) {
        sum_ = b;
        state_ = kMid;
      } else if (b != kPreamble) {
        state_ = kPre;
      }
      return false;
    case kMid:
      mid_ = b;
      sum_ = static_cast<uint8_t>(sum_ + b);
      state_ = kLen;
      return false;
    case kLen:
      if (b == 0xFF) {  // extended length: not used by the node; look for the next preamble
        state_ = kPre;
        return false;
      }
      sum_ = static_cast<uint8_t>(sum_ + b);
      pos_ = 0;
      len_ = b;
      state_ = len_ ? kData : kSum;
      return false;
    case kData:
      sum_ = static_cast<uint8_t>(sum_ + b);
      buf_[pos_] = b;
      if (++pos_ == len_) state_ = kSum;
      return false;
    case kSum:
      sum_ = static_cast<uint8_t>(sum_ + b);
      state_ = kPre;
      if (sum_ != 0) {
        checksum_errors_++;
        return false;
      }
      msg.mid = mid_;
      msg.len = len_;
      msg.data = buf_;
      return true;
  }
  state_ = kPre;
  return false;
}

bool parse_mtdata2(const Message &m, Sample &s) {
  s.has_euler = s.has_acc = s.has_gyro = false;
  if (m.mid != kMidMtData2) return false;
  uint16_t i = 0;
  while (i < m.len) {
    if (m.len - i < 3) return false;
    const uint16_t id = static_cast<uint16_t>((m.data[i] << 8) | m.data[i + 1]);
    const uint8_t size = m.data[i + 2];
    i = static_cast<uint16_t>(i + 3);
    if (size > m.len - i) return false;
    if (size == 12) {
      if (id == kXdiEulerAngles) {
        memcpy(s.euler, m.data + i, 12);
        s.has_euler = true;
      } else if (id == kXdiAcceleration) {
        memcpy(s.acc, m.data + i, 12);
        s.has_acc = true;
      } else if (id == kXdiRateOfTurn) {
        memcpy(s.gyro, m.data + i, 12);
        s.has_gyro = true;
      }
    }
    i = static_cast<uint16_t>(i + size);
  }
  return true;
}

size_t output_config(uint16_t rate_hz, uint8_t *out, size_t cap) {
  static const uint16_t ids[] = {kXdiEulerAngles, kXdiAcceleration, kXdiRateOfTurn};
  if (cap < 12) return 0;
  for (size_t k = 0; k < 3; k++) {
    out[4 * k + 0] = static_cast<uint8_t>(ids[k] >> 8);
    out[4 * k + 1] = static_cast<uint8_t>(ids[k] & 0xFF);
    out[4 * k + 2] = static_cast<uint8_t>(rate_hz >> 8);
    out[4 * k + 3] = static_cast<uint8_t>(rate_hz & 0xFF);
  }
  return 12;
}

}  // namespace xbus

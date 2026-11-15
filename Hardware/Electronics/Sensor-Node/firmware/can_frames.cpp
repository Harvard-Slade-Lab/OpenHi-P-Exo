#include "can_frames.h"

#include <string.h>

namespace frames {

namespace {

Frame make(uint16_t id, const uint8_t *first, const uint8_t *second) {
  Frame f;
  f.id = id;
  memset(f.data, 0, sizeof f.data);
  memcpy(f.data, first, 4);
  if (second) memcpy(f.data + 4, second, 4);
  return f;
}

}  // namespace

Frame load(uint16_t base, int32_t count) {
  const uint32_t u = static_cast<uint32_t>(count);
  const uint8_t le[4] = {static_cast<uint8_t>(u), static_cast<uint8_t>(u >> 8),
                         static_cast<uint8_t>(u >> 16), static_cast<uint8_t>(u >> 24)};
  return make(static_cast<uint16_t>(base + kLoad), le, nullptr);
}

int imu(uint16_t base, const xbus::Sample &s, Frame out[6]) {
  int n = 0;
  if (s.has_euler) {  // the MTi's float32 values are big-endian, as the controller expects
    out[n++] = make(static_cast<uint16_t>(base + kRollPitch), s.euler, s.euler + 4);
    out[n++] = make(static_cast<uint16_t>(base + kYaw), s.euler + 8, nullptr);
  }
  if (s.has_acc) {
    out[n++] = make(static_cast<uint16_t>(base + kAccXY), s.acc, s.acc + 4);
    out[n++] = make(static_cast<uint16_t>(base + kAccZ), s.acc + 8, nullptr);
  }
  if (s.has_gyro) {
    out[n++] = make(static_cast<uint16_t>(base + kGyroXY), s.gyro, s.gyro + 4);
    out[n++] = make(static_cast<uint16_t>(base + kGyroZ), s.gyro + 8, nullptr);
  }
  return n;
}

}  // namespace frames

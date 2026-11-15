// CAN1 frames that the controller decodes (Controller/Core/Src/imu_processing.c, can_rx_callback.c).
// 11-bit identifiers: base 0x300 (left leg) or 0x500 (right leg) plus the low byte below
// (Controller/Core/Inc/globals.h: IMU_LEFT_ID, IMU_RIGHT_ID). Every frame has 8 data bytes;
// bytes that carry no value are 0.
#pragma once

#include <stdint.h>

#include "xbus.h"

namespace frames {

constexpr uint16_t kLeftBase = 0x300;
constexpr uint16_t kRightBase = 0x500;

constexpr uint16_t kLoad = 0x10;       // bytes 0-3: load-cell ADC count, int32 little-endian
constexpr uint16_t kRollPitch = 0x11;  // bytes 0-3 roll, 4-7 pitch: float32 big-endian
constexpr uint16_t kYaw = 0x12;        // bytes 0-3 yaw
constexpr uint16_t kAccXY = 0x13;      // bytes 0-3 accX, 4-7 accY
constexpr uint16_t kAccZ = 0x14;       // bytes 0-3 accZ
constexpr uint16_t kGyroXY = 0x15;     // bytes 0-3 gyroX, 4-7 gyroY
constexpr uint16_t kGyroZ = 0x16;      // bytes 0-3 gyroZ

struct Frame {
  uint16_t id;
  uint8_t data[8];
};

Frame load(uint16_t base, int32_t count);

// Frames for the values present in s (2 per value group, up to 6). Returns the number written.
int imu(uint16_t base, const xbus::Sample &s, Frame out[6]);

}  // namespace frames

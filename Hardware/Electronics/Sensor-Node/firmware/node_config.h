// Settings of the sensor node. Set NODE_LEG before uploading: the left node sends the CAN1 frames
// 0x310-0x316, the right node 0x510-0x516.
#pragma once

#include <stdint.h>

#define LEG_LEFT 0
#define LEG_RIGHT 1

#ifndef NODE_LEG
#define NODE_LEG LEG_RIGHT
#endif

namespace node {

constexpr uint32_t kCanBitRate = 1000000;  // CAN1 of the controller: 1 Mbit/s (Controller/Core/Src/can.c)
constexpr uint32_t kLoadPeriodUs = 2000;   // one load-cell frame every 2 ms (500 Hz)
constexpr uint32_t kLoadAverage = 4;       // ADC readings averaged per load-cell frame
constexpr uint16_t kImuRateHz = 100;       // MTi output rate; the MTi 1-series outputs 0-100 Hz (MT0512P)
constexpr uint32_t kImuSilenceMs = 1000;   // no MTi data for this long: configure the MTi again
constexpr uint32_t kRateCheckMs = 15;      // a stored output is kept only if messages come this close
constexpr uint32_t kAckTimeoutMs = 40;     // wait for the acknowledgement of GoToConfig at one speed
constexpr uint32_t kReplyTimeoutMs = 250;  // wait for the other MTi replies
constexpr uint32_t kByteGapUs = 2000;      // a pause this long inside a message means it was cut
constexpr uint32_t kPrintPeriodMs = 100;   // status line on USB while a serial monitor is open

// UART speeds tried in turn until the MTi acknowledges GoToConfig; 921.6 kbit/s is the typical
// speed in the MTi 1-series data sheet (MT0512P, Table 8). UART 8N1 (MT0512P 1.7).
constexpr uint32_t kMtiBaud[] = {921600, 115200, 230400, 460800, 2000000};

}  // namespace node

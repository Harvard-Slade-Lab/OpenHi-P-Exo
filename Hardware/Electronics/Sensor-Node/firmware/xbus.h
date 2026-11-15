// Xbus messages of the Xsens MTi 1-series (MTi-3) over UART.
//
// Message (standard length): preamble 0xFA, bus identifier 0xFF, message ID, LEN (0-254), LEN data
// bytes, checksum. The checksum makes the sum of every byte after the preamble 0 (mod 256).
// The node does not use extended-length messages (LEN 0xFF): the parser drops such a header and
// looks for the next preamble.
// Sources: MTi 1-series data sheet MT0512P, section 6 (message structure, SetOutputConfiguration
// with data, MTData2 and its data identifiers); GoToConfig, GoToMeasurement, the request form of the
// output configuration (0xC0 without data) and the acknowledgements are from Xsens's MT Low Level
// Communication Protocol documentation (LLCP), to which the data sheet refers for the full set.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace xbus {

constexpr uint8_t kPreamble = 0xFA;
constexpr uint8_t kBusId = 0xFF;

constexpr uint8_t kMidGoToConfig = 0x30;
constexpr uint8_t kMidGoToConfigAck = 0x31;
constexpr uint8_t kMidGoToMeasurement = 0x10;
constexpr uint8_t kMidGoToMeasurementAck = 0x11;
constexpr uint8_t kMidOutputConfig = 0xC0;     // with data: set (MT0512P 6.2); without data: request (LLCP)
constexpr uint8_t kMidOutputConfigAck = 0xC1;  // the configuration in use (LLCP)
constexpr uint8_t kMidMtData2 = 0x36;          // MT0512P 6.3

// MTData2 data identifiers (MT0512P 6.2): format bits 0x0 = float32, coordinate system 0x0 = ENU.
constexpr uint16_t kXdiEulerAngles = 0x2030;   // roll, pitch, yaw
constexpr uint16_t kXdiAcceleration = 0x4020;  // accX, accY, accZ
constexpr uint16_t kXdiRateOfTurn = 0x8020;    // gyrX, gyrY, gyrZ

// Writes a standard-length message into out. Returns its length (len + 5), or 0 if it does not fit.
size_t build(uint8_t mid, const uint8_t *data, uint8_t len, uint8_t *out, size_t cap);

struct Message {
  uint8_t mid;
  uint16_t len;
  const uint8_t *data;  // points into the parser; valid until the next call of Parser::feed
};

// Byte-by-byte parser. feed() returns true when a complete standard-length message with a valid
// checksum has arrived; msg then describes it. reset() drops a message cut short.
class Parser {
 public:
  bool feed(uint8_t b, Message &msg);
  bool idle() const { return state_ == kPre; }
  void reset() { state_ = kPre; }
  uint32_t checksum_errors() const { return checksum_errors_; }

 private:
  enum State { kPre, kBid, kMid, kLen, kData, kSum };
  State state_ = kPre;
  uint8_t mid_ = 0;
  uint16_t len_ = 0;
  uint16_t pos_ = 0;
  uint8_t sum_ = 0;
  uint32_t checksum_errors_ = 0;
  uint8_t buf_[254];
};

// Inertial values of one MTData2 message, kept as the MTi sent them (float32, big-endian, 4 bytes
// per value).
struct Sample {
  bool has_euler = false;
  bool has_acc = false;
  bool has_gyro = false;
  uint8_t euler[12];  // roll, pitch, yaw (degrees)
  uint8_t acc[12];    // accX, accY, accZ (m/s^2)
  uint8_t gyro[12];   // gyrX, gyrY, gyrZ (rad/s)
};

// Reads the packets of an MTData2 message (data identifier, size, data). Returns false if a packet
// runs past the end of the message or the message is not MTData2.
bool parse_mtdata2(const Message &m, Sample &s);

// Output configuration used by the node: Euler angles, acceleration and rate of turn, each at
// rate_hz. Writes 12 bytes (data identifier and frequency, both 16 bit big-endian, per entry).
size_t output_config(uint16_t rate_hz, uint8_t *out, size_t cap);

}  // namespace xbus

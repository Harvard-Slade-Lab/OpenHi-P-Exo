// Sensor node firmware: Adafruit Feather M4 CAN Express (ATSAME51) with an Xsens MTi-3 on Serial1
// and the load-cell amplifier output on A0. Sends the CAN1 frames that the controller decodes
// (see can_frames.h).
//
// - MTi-3: at power-up the node listens for 300 ms. If the MTi already sends Euler angles,
//   acceleration and rate of turn about every 10 ms (its output configuration is kept in its
//   non-volatile memory), the node uses them as they are. Otherwise, and whenever the MTi has sent
//   no roll and gyro data for 1 s, the node puts the MTi in configuration mode (GoToConfig at each
//   UART speed in turn), sets that output at 100 Hz only if the stored configuration differs, and
//   starts measurement. Each MTData2 message gives the frames 0x_11-0x_16 at once, with the MTi's
//   float32 values copied unchanged.
// - Load cell: every 2 ms the node averages four 12-bit readings of A0 and sends frame 0x_10,
//   from power-up on and also while it waits for the MTi.
// - A frame is sent only with a new value from its sensor: if the MTi stops, its frames stop and
//   the controller stops assistance after 100 ms (Controller/Core/Inc/fault_monitor.h).
// - LED (D13): on while roll and gyro data arrive, off 1 s after the last.
// - USB: while a serial monitor is open (115200 baud), a line every 0.1 s with roll (degrees), gyroX
//   (rad/s) and the load count, for the bench checks. The node detects the monitor from the port's
//   DTR signal, a check that does not wait, and skips a line when the port reports fewer than
//   48 bytes free, so that the frames keep their timing.
//
// Arduino IDE: board "Adafruit Feather M4 CAN (SAME51)" (Adafruit SAMD Boards), library
// "Adafruit CAN" (CANSAME5x). Set NODE_LEG in node_config.h first.

#include <CANSAME5x.h>
#include <stdio.h>
#include <string.h>

#include "can_frames.h"
#include "node_config.h"
#include "xbus.h"

static CANSAME5x CAN;
static xbus::Parser parser;
static const uint16_t kBase = (NODE_LEG == LEG_LEFT) ? frames::kLeftBase : frames::kRightBase;
static uint32_t mtiBaud = node::kMtiBaud[0];  // the UART speed that worked last
static bool uartOpen = false;
static uint32_t nextLoadUs;
static uint32_t lastByteUs;      // time of the last byte from the MTi
static bool haveData = false;    // roll and gyro data have arrived at least once
static uint32_t lastDataMs;      // time of the last MTData2 message with roll and gyro data
static uint32_t lastTryMs;       // end of the last configuration attempt
static int32_t lastCount = -1;   // last load count sent
static float lastRoll, lastGyroX;
static uint32_t lastPrintMs;

static void sendFrame(const frames::Frame &f) {
  CAN.beginPacket(f.id);  // 11-bit standard identifier
  CAN.write(f.data, sizeof f.data);
  CAN.endPacket();
}

// Sends a load-cell frame when one is due (every kLoadPeriodUs).
static void serviceLoad() {
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextLoadUs) < 0) return;
  nextLoadUs += node::kLoadPeriodUs;
  if (static_cast<int32_t>(nowUs - nextLoadUs) >= 0) nextLoadUs = nowUs + node::kLoadPeriodUs;  // late: resync
  uint32_t sum = 0;
  for (uint32_t i = 0; i < node::kLoadAverage; i++) sum += static_cast<uint32_t>(analogRead(A0));
  lastCount = static_cast<int32_t>((sum + node::kLoadAverage / 2) / node::kLoadAverage);
  sendFrame(frames::load(kBase, lastCount));
}

// float32 sent big-endian by the MTi
static float beFloat(const uint8_t *b) {
  const uint32_t u = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                     (static_cast<uint32_t>(b[2]) << 8) | b[3];
  float v;
  memcpy(&v, &u, sizeof v);
  return v;
}

// v with 'decimals' decimals, without printf's %f
static void fixed(char *buf, size_t n, float v, int decimals) {
  long scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;
  long r = static_cast<long>(v * scale + (v < 0 ? -0.5f : 0.5f));
  const char *sign = r < 0 ? "-" : "";
  if (r < 0) r = -r;
  snprintf(buf, n, "%s%ld.%0*ld", sign, r / scale, decimals, r % scale);
}

// Status line on USB while a serial monitor is open: roll, gyroX and the load count.
static void printStatus(bool streaming) {
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs < node::kPrintPeriodMs) return;
  lastPrintMs = nowMs;
  // DTR is set while a monitor holds the port open, and reading it does not wait; 'if (Serial)'
  // waits 10 ms on each call with the core's default USB stack. 48 bytes: the longest line
  // ("roll -180.0 deg, gyroX -34.91 rad/s, load 4095") with CR LF; that stack reports 63 bytes free.
  if (!Serial.dtr() || Serial.availableForWrite() < 48) return;
  char line[72];
  if (streaming) {
    char roll[16], gyro[16];
    fixed(roll, sizeof roll, lastRoll, 1);
    fixed(gyro, sizeof gyro, lastGyroX, 2);
    snprintf(line, sizeof line, "roll %s deg, gyroX %s rad/s, load %ld", roll, gyro, static_cast<long>(lastCount));
  } else {
    snprintf(line, sizeof line, "MTi: no data, load %ld", static_cast<long>(lastCount));
  }
  Serial.println(line);
}

static void openUart(uint32_t baud) {
  if (uartOpen) Serial1.end();
  Serial1.begin(baud);  // 8N1
  uartOpen = true;
  parser.reset();
}

// Feeds the next received byte to the parser. The MTi sends a message without pauses, so a pause
// of kByteGapUs inside a message means that it was cut (loose wire, MTi reset): start over.
static bool feedMti(xbus::Message &m) {
  const uint32_t nowUs = micros();
  if (!parser.idle() && nowUs - lastByteUs > node::kByteGapUs) parser.reset();
  lastByteUs = nowUs;
  return parser.feed(static_cast<uint8_t>(Serial1.read()), m);
}

static void sendToMti(uint8_t mid, const uint8_t *data, uint8_t len) {
  uint8_t buf[32];
  const size_t n = xbus::build(mid, data, len, buf, sizeof buf);
  Serial1.write(buf, n);
  Serial1.flush();
}

// Reads Serial1 until a message with ID 'mid' arrives or the time runs out. Other messages
// (MTData2 sent before the MTi left measurement mode) are dropped.
static bool waitForMti(uint8_t mid, uint32_t timeoutMs, xbus::Message *reply) {
  const uint32_t t0 = millis();
  xbus::Message m;
  while (millis() - t0 < timeoutMs) {
    serviceLoad();
    while (Serial1.available() > 0) {
      if (feedMti(m) && m.mid == mid) {
        if (reply) *reply = m;
        return true;
      }
    }
  }
  return false;
}

// True if two MTData2 messages with all three outputs arrive within kRateCheckMs of each other
// (the MTi already sends them at about 100 Hz) before timeoutMs.
static bool mtiStreams(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  bool seen = false;
  uint32_t lastMs = 0;
  xbus::Message m;
  xbus::Sample s;
  while (millis() - t0 < timeoutMs) {
    serviceLoad();
    while (Serial1.available() > 0) {
      if (!feedMti(m) || !xbus::parse_mtdata2(m, s) || !s.has_euler || !s.has_acc || !s.has_gyro) continue;
      const uint32_t nowMs = millis();
      if (seen && nowMs - lastMs <= node::kRateCheckMs) return true;
      seen = true;
      lastMs = nowMs;
    }
  }
  return false;
}

// GoToConfig at each UART speed in turn (the speed that worked last first), kAckTimeoutMs each,
// in rounds until the MTi answers or patienceMs has passed.
static bool enterConfig(uint32_t patienceMs) {
  const size_t nBaud = sizeof node::kMtiBaud / sizeof node::kMtiBaud[0];
  const uint32_t t0 = millis();
  do {
    for (size_t k = 0; k <= nBaud; k++) {
      const uint32_t baud = (k == 0) ? mtiBaud : node::kMtiBaud[k - 1];
      if (k > 0 && baud == mtiBaud) continue;
      openUart(baud);
      sendToMti(xbus::kMidGoToConfig, nullptr, 0);
      if (waitForMti(xbus::kMidGoToConfigAck, node::kAckTimeoutMs, nullptr)) {
        mtiBaud = baud;
        return true;
      }
    }
  } while (millis() - t0 < patienceMs);
  openUart(mtiBaud);
  return false;
}

static bool configureMti(uint32_t patienceMs) {
  if (!enterConfig(patienceMs)) return false;
  uint8_t want[12];
  const size_t n = xbus::output_config(node::kImuRateHz, want, sizeof want);
  // The output configuration is stored in the MTi; write it only if it differs.
  xbus::Message current;
  sendToMti(xbus::kMidOutputConfig, nullptr, 0);
  const bool same = waitForMti(xbus::kMidOutputConfigAck, node::kReplyTimeoutMs, &current) &&
                    current.len == n && memcmp(current.data, want, n) == 0;
  if (!same) {
    sendToMti(xbus::kMidOutputConfig, want, static_cast<uint8_t>(n));
    if (!waitForMti(xbus::kMidOutputConfigAck, 4 * node::kReplyTimeoutMs, nullptr)) return false;
  }
  sendToMti(xbus::kMidGoToMeasurement, nullptr, 0);
  return waitForMti(xbus::kMidGoToMeasurementAck, node::kReplyTimeoutMs, nullptr);
}

void setup() {
  Serial.begin(115200);  // USB, status messages only
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);  // CAN transceiver active
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, HIGH);  // 5 V for the CAN transceiver
  analogReadResolution(12);             // 0-4095 over 0-3.3 V, as the controller assumes
  while (!CAN.begin(node::kCanBitRate)) delay(100);
  nextLoadUs = micros();  // load-cell frames from here on
  openUart(mtiBaud);
  const bool ok = mtiStreams(300) || configureMti(1500);
  if (Serial.dtr()) Serial.println(ok ? "MTi streaming" : "MTi not answering; retrying");
  lastTryMs = millis();
}

void loop() {
  serviceLoad();

  xbus::Message m;
  while (Serial1.available() > 0) {
    if (!feedMti(m) || m.mid != xbus::kMidMtData2) continue;
    xbus::Sample s;
    if (!xbus::parse_mtdata2(m, s)) continue;
    frames::Frame f[6];
    const int count = frames::imu(kBase, s, f);
    for (int i = 0; i < count; i++) sendFrame(f[i]);
    if (s.has_euler && s.has_gyro) {  // the frames the controller checks (0x_11, 0x_15)
      haveData = true;
      lastDataMs = millis();
      lastRoll = beFloat(s.euler);
      lastGyroX = beFloat(s.gyro);
    }
    break;  // one message per pass, so that the load-cell frames stay on time
  }

  const uint32_t nowMs = millis();
  const bool streaming = haveData && nowMs - lastDataMs <= node::kImuSilenceMs;
  digitalWrite(LED_BUILTIN, streaming ? HIGH : LOW);
  printStatus(streaming);
  if (!streaming && nowMs - lastTryMs > node::kImuSilenceMs) {  // configure the MTi again
    configureMti(500);
    lastTryMs = millis();
  }
}

#ifndef INC_FAULT_MONITOR_H_
#define INC_FAULT_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

// Fault codes: bits of FaultMonitor.faults and of TxData.faultCode (SD log).
// A fault stays set until reset. Every fault except FAULT_CAN1_ERROR and FAULT_CAN2_ERROR
// stops assistance: both motors get zero current until reset.
#define FAULT_LIMIT (1UL << 0)             // motor current, measured torque or motor temperature limit
#define FAULT_IMU_LEFT_STALE (1UL << 1)    // no roll (0x311) or gyro (0x315) frame for FAULT_STALE_MS
#define FAULT_IMU_RIGHT_STALE (1UL << 2)   // no roll (0x511) or gyro (0x515) frame for FAULT_STALE_MS
#define FAULT_LOAD_LEFT_STALE (1UL << 3)   // no load-cell frame (0x310) for FAULT_STALE_MS
#define FAULT_LOAD_RIGHT_STALE (1UL << 4)  // no load-cell frame (0x510) for FAULT_STALE_MS
#define FAULT_LOAD_LEFT_RANGE (1UL << 5)   // load-cell value outside FAULT_LOAD_MIN..FAULT_LOAD_MAX
#define FAULT_LOAD_RIGHT_RANGE (1UL << 6)
#define FAULT_LOAD_LEFT_STUCK (1UL << 7)   // identical load-cell value for FAULT_LOAD_STUCK_MS during assistance
#define FAULT_LOAD_RIGHT_STUCK (1UL << 8)
#define FAULT_MOTOR_LEFT_STALE (1UL << 9)  // no motor feedback frame for FAULT_STALE_MS
#define FAULT_MOTOR_RIGHT_STALE (1UL << 10)
#define FAULT_MOTOR_LEFT_ERROR (1UL << 11) // nonzero motor error code (servo_receiving.h)
#define FAULT_MOTOR_RIGHT_ERROR (1UL << 12)
#define FAULT_MOTOR_TX (1UL << 13)         // a motor command could not be queued on CAN2
#define FAULT_CAN1_BUS_OFF (1UL << 14)     // sensor bus
#define FAULT_CAN2_BUS_OFF (1UL << 15)     // motor bus
#define FAULT_CAN1_ERROR (1UL << 16)       // error warning, error passive or bus-off: logged only
#define FAULT_CAN2_ERROR (1UL << 17)
#define FAULT_WATCHDOG_RESET (1UL << 18)   // the last reset was caused by the independent watchdog

#define FAULT_STOP_MASK (~(FAULT_CAN1_ERROR | FAULT_CAN2_ERROR))

#define FAULT_STALE_MS 100      // CAN feedback older than this is stale
#define FAULT_LOAD_STUCK_MS 500 // load-cell value unchanged this long while assisting is stuck
#define FAULT_LOAD_MIN 1.0f     // load-cell ADC count (12 bit); 0 and 4095 are the saturated ends
#define FAULT_LOAD_MAX 4094.0f

typedef enum
{
    FAULT_SIDE_LEFT = 0,
    FAULT_SIDE_RIGHT = 1,
} FaultSide;

typedef enum
{
    FAULT_STREAM_ROLL = 0, // CAN1 0x311 / 0x511
    FAULT_STREAM_GYRO,     // CAN1 0x315 / 0x515
    FAULT_STREAM_LOAD,     // CAN1 0x310 / 0x510
    FAULT_STREAM_MOTOR,    // CAN2 motor feedback
    FAULT_STREAM_COUNT,
} FaultStream;

typedef struct
{
    uint32_t faults;
    uint32_t lastRxMs[2][FAULT_STREAM_COUNT];
    float loadLast[2];
    uint32_t loadChangeMs[2];
    int8_t motorError[2]; // last motor error code
} FaultMonitor;

void fault_monitor_init(FaultMonitor *m, uint32_t nowMs);
void fault_monitor_set(FaultMonitor *m, uint32_t faults);

// CAN receive interrupt: a sensor frame (CAN1 standard ID) or a motor feedback frame (CAN2)
void fault_monitor_sensor_frame(FaultMonitor *m, uint32_t stdId, uint32_t nowMs);
void fault_monitor_motor_frame(FaultMonitor *m, FaultSide side, int8_t errorCode, uint32_t nowMs);

// CAN error callback: bus 1 or 2
void fault_monitor_can_error(FaultMonitor *m, int bus, bool busOff);

// Control loop (500 Hz): checks freshness and load-cell plausibility. Returns true while
// assistance must stay off (zero current).
bool fault_monitor_check(FaultMonitor *m, uint32_t nowMs, float loadLeft, float loadRight, bool assisting);

#endif /* INC_FAULT_MONITOR_H_ */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include <stdint.h>

#define MOTOR_COUNT 2

extern int motor_ids[MOTOR_COUNT];

#define IMU_LEFT_ID 0x300
#define IMU_RIGHT_ID 0x500

// BLE RX ring buffer (USART1 circular DMA), parsed in the main loop (Bluetooth protocol, ble_protocol.h)
#define BLE_RX_RING_SIZE 256

typedef struct
{
    float Position;
    float Speed;
    float Current;
    float Temp;
} MotorData;

typedef struct
{
    float load;
    float roll, pitch, yaw;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
} IMUData;

typedef struct
{
    MotorData motor;
    IMUData imu;
    float gaitCycleNorm;
    float torqueDesired;
    float torqueMeasured;
} LegData;

// SD log record (148 bytes, little-endian), written as 0xAA 0xAA + record + 0xBB 0xBB at 100 Hz.
// Layout: 140 bytes of motion and sensor data (time, both legs) followed by the fault fields.
typedef struct
{
    float timeMsec;
    LegData rightLeg;
    LegData leftLeg;
    uint32_t faultCode;     // FAULT_* bits (fault_monitor.h)
    int8_t motorErrorRight; // last motor error code (servo_receiving.h)
    int8_t motorErrorLeft;
    uint16_t reserved;
} TxData;

#define NUM_POINTS 5
#define NUM_SEGMENTS (NUM_POINTS - 1)

typedef struct
{
    float extensionPeakTorque;
    float flexionPeakTorque;
    float extensionPeakTime;
    float flexionPeakTime;
    float midTime;
    float extensionRiseTime;

    float x[NUM_POINTS];
    float y[NUM_POINTS];
    float d[NUM_POINTS];
    float c0[NUM_SEGMENTS], c1[NUM_SEGMENTS], c2[NUM_SEGMENTS], c3[NUM_SEGMENTS];
} TorqueProfile;

#endif /* INC_GLOBALS_H_ */

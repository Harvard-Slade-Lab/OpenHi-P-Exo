#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#define MOTOR_COUNT 2

extern int motor_ids[MOTOR_COUNT];

#define IMU_LEFT_ID 0x300
#define IMU_RIGHT_ID 0x500

// BLE RX Data Size
#define RX_BYTES 28 // 6 params * 4 bytes + Padding (start, end): 4 bytes = 28 bytes

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

typedef struct
{
    float timeMsec;
    LegData rightLeg;
    LegData leftLeg;
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

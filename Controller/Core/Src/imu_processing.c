#include <string.h>
#include "imu_processing.h"

#define IMU_DATA0 0x010 // Loadcell
#define IMU_DATA1 0x011 // Roll, Pitch
#define IMU_DATA2 0x012 // Yaw
#define IMU_DATA3 0x013 // AccX, AccY
#define IMU_DATA4 0x014 // AccZ
#define IMU_DATA5 0x015 // GyroX, GyroY
#define IMU_DATA6 0x016 // GyroZ

void process_imu_data(IMUData *imu, uint32_t stdId, uint8_t *data)
{
    uint32_t imuType = stdId & 0x0FF; // Isolate the lower three digits

    switch (imuType)
    {
    case IMU_DATA0: // Loadcell
        imu->load = convert_bytes_to_float_little(&data[0]);
        break;
    case IMU_DATA1: // Roll, Pitch
        imu->roll = convert_bytes_to_float(&data[0]);
        imu->pitch = convert_bytes_to_float(&data[4]);
        break;

    case IMU_DATA2: // Yaw
        imu->yaw = convert_bytes_to_float(&data[0]);
        break;

    case IMU_DATA3: // AccX, AccY
        imu->accX = convert_bytes_to_float(&data[0]);
        imu->accY = convert_bytes_to_float(&data[4]);
        break;

    case IMU_DATA4: // AccZ
        imu->accZ = convert_bytes_to_float(&data[0]);
        break;

    case IMU_DATA5: // GyroX, GyroY
        imu->gyroX = convert_bytes_to_float(&data[0]);
        imu->gyroY = convert_bytes_to_float(&data[4]);
        break;

    case IMU_DATA6: // GyroZ
        imu->gyroZ = convert_bytes_to_float(&data[0]);
        break;
    }
}

float convert_bytes_to_float(uint8_t *bytes) // Sender: float pack (BE)
{
    uint32_t temp = ((uint32_t)bytes[0] << 24) |
                    ((uint32_t)bytes[1] << 16) |
                    ((uint32_t)bytes[2] << 8)  |
                    ((uint32_t)bytes[3]);
    float f;
    memcpy(&f, &temp, sizeof(f));
    return f;
}

float convert_bytes_to_float_little(uint8_t *bytes) // Sender: int32_t pack (LE)
{
    int32_t temp = (int32_t)(((uint32_t)bytes[3] << 24) |
                            ((uint32_t)bytes[2] << 16) |
                            ((uint32_t)bytes[1] << 8)  |
                            ((uint32_t)bytes[0]));
    return (float)temp;
}
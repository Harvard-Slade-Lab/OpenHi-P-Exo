#ifndef INC_IMU_PROCESSING_H_
#define INC_IMU_PROCESSING_H_

#include <stdint.h>
#include "globals.h"

void process_imu_data(IMUData *imu, uint32_t stdId, uint8_t *data);
float convert_bytes_to_float(uint8_t *bytes);
float convert_bytes_to_float_little(uint8_t *bytes);

#endif /* INC_IMU_PROCESSING_H_ */
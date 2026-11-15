#ifndef INC_CONTROL_LOGIC_H_
#define INC_CONTROL_LOGIC_H_

#include "globals.h"

void update_control_mode(uint8_t *controlMode);

void main_controller(float timeSec,
					 float imuOffsetLeft, float imuOffsetRight,
					 IMUData *imuDataLeft, IMUData *imuDataRight,
					 float *gaitCycleNormLeft, float *gaitCycleNormRight,
					 bool *walkingFlagLeft, bool *walkingFlagRight,
					 TorqueProfile *profile,
					 float *torqueDesiredLeft, float *torqueDesiredRight);

#endif /* INC_CONTROL_LOGIC_H_ */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "motor_control.h"
#include "servo_sending.h"

extern float G;
extern float k_t;
extern float k_p;
extern float k_d;

void motor_current_control(uint8_t motorID, float torqueDesired, float torqueMeasured,
						   float motorSpeed, bool isLeftFlag)
{
	float motorCurCommand = 0.0f;
	if (isLeftFlag)
		motorCurCommand = -(torqueDesired / (k_t * G)) - k_p * (torqueDesired - torqueMeasured) - k_d * motorSpeed / 1000.0f;
	else
		motorCurCommand = (torqueDesired / (k_t * G)) + k_p * (torqueDesired - torqueMeasured) - k_d * motorSpeed / 1000.0f;

	comm_can_set_current(motorID, motorCurCommand);
}
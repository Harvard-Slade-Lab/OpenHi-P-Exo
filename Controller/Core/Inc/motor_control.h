#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void motor_current_control(uint8_t motorID, float torqueDesired, float torqueMeasured,
                           float motorSpeed, bool isLeftFlag);

#endif // MOTOR_CONTROL_H
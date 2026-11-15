#ifndef __SERVO_RECEIVING_H__
#define __SERVO_RECEIVING_H__

#include <stdint.h>

typedef enum {
    NO_ERROR = 0,
    OVER_TEMP_FAULT,
    OVER_CURRENT_FAULT,
    OVER_VOLTAGE_FAULT,
    UNDER_VOLTAGE_FAULT,
    ENCODER_FAULT,
    PHASE_CURRENT_UNBALANCE_FAULT,
  } ERROR_CODES;

extern float motor_pos;
extern float motor_spd;
extern float motor_cur;
extern float motor_temp;
extern int8_t motor_error;

void motor_receive(uint8_t* rx_message);

#endif

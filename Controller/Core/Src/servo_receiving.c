#include "servo_receiving.h"

float motor_pos;
float motor_spd;
float motor_cur;
float motor_temp;
int8_t motor_error;

// can receive protocol
void motor_receive(uint8_t *rx_message)
{
  int16_t pos_int = rx_message[0] << 8 | rx_message[1];
  int16_t spd_int = rx_message[2] << 8 | rx_message[3];
  int16_t cur_int = rx_message[4] << 8 | rx_message[5];
  motor_pos = (float)(pos_int * 0.1f);  // motor position [Degree]
  motor_spd = (float)(spd_int * 10.0f); // motor speed [ERPM]
  motor_cur = (float)(cur_int * 0.01f); // motor current [A]
  motor_temp = (float)rx_message[6];    // motor temperature [Celsius]
  motor_error = rx_message[7];          // motor error mode
}
/*
 * ERROR CODE
 * 0: no fault
 * 1: over temperature fault
 * 2: over current fault
 * 3: over voltage fault
 * 4: under voltage fault
 * 5: encoder fault
 * 6: phase current unbalance fault (The hardware may be damaged)
 */

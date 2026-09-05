#ifndef __SERVO_SENDING_H__
#define __SERVO_SENDING_H__

#include <stdint.h>

//**************************CAN_PACKET_ID*************************//
//datasheet
typedef enum {
  CAN_PACKET_SET_DUTY = 0,      // Duty cycle mode
  CAN_PACKET_SET_CURRENT,       // Current loop mode
  CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
  CAN_PACKET_SET_RPM,           // Velocity mode
  CAN_PACKET_SET_POS,           // Position mode
  CAN_PACKET_SET_ORIGIN_HERE,   // Set origin mode
  CAN_PACKET_SET_POS_SPD,       // Position velocity loop mode
  } CAN_PACKET_ID;

void sendFrameEXTId(uint16_t ID, uint8_t* data, uint8_t len);
void servo_motor_initialize(uint16_t ID); //enter_motor_mode_servo + set_origin
void enter_motor_mode_servo(uint8_t controller_id);
void exit_motor_mode_servo(uint8_t controller_id);

void comm_can_set_duty(uint8_t controller_id, float duty); // a certain duty cycle voltage to motor.
void comm_can_set_current(uint8_t controller_id, float current); // a Iq current to motor, the motor output torque = Iq *KT, so it can be used as a torque loop
void comm_can_set_cb(uint8_t controller_id, float current); // a certain brake current to motor, in order to fasten the motor at the current position (pay attention to the motor temperature when using)
void comm_can_set_rpm(uint8_t controller_id, float rpm); // a certain motion speed to motor
void comm_can_set_pos(uint8_t controller_id, float pos); // a certain position to motor, the motor will run to the specified position, (default speed 12000erpm acceleration 40000erpm)
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode); 
void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ); // a certain position, speed and acceleration to motor. The motor will run at a given acceleration and maximum speed to a specified position.

#endif

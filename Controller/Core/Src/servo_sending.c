#include "servo_sending.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan2; // This is a global variable that represents the CAN1 peripheral.
CAN_TxHeaderTypeDef Tx0Header;  // Structure that defines the parameters of a CAN Tx message header.
uint32_t Tx0Mailbox;            // Mailbox identifier

extern uint8_t buffer[8]; // Buffer for CAN data

// Sends a CAN frame with a given ID, data, and data length. This function is often used in the context of CANopen protocols.
// The ID is the identifier of the message (which usually indicates its purpose) and the data is an array of bytes to be sent.
// The function first sets the various properties of the CAN message header, and then sends the message using the HAL_CAN_AddTxMessage function.
// If all the transmit mailboxes are full, it waits until there is space to transmit.

void sendFrameEXTId(uint16_t ID, uint8_t *data, uint8_t len)
{

  // Setting the CAN message properties
  Tx0Header.ExtId = ID;
  Tx0Header.RTR = CAN_RTR_DATA;
  Tx0Header.IDE = CAN_ID_EXT;
  Tx0Header.DLC = len;

  // Waiting for a free Tx mailbox
  uint32_t startTime = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
  {
    if ((HAL_GetTick() - startTime) > 20)
    {
      return;
    }
  }
  // Sending the message
  HAL_CAN_AddTxMessage(&hcan2, &Tx0Header, data, &Tx0Mailbox);

  // Waiting for the message to be transmitted
  startTime = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan2, Tx0Mailbox) == 1)
  {
    if ((HAL_GetTick() - startTime) > 20)
    {
      return;
    }
  }
}

// These functions are used to configure a motor to enter or exit servo mode.
// Reference: https://tmotorcancontrol.readthedocs.io/en/latest/_modules/TMotorCANControl/servo_can.html#servo_command.__init__
void servo_motor_initialize(uint16_t ID)
{
  enter_motor_mode_servo(ID);
  comm_can_set_origin(ID, 0);
}

void enter_motor_mode_servo(uint8_t controller_id)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFF;
  buffer[3] = 0xFF;
  buffer[4] = 0xFF;
  buffer[5] = 0xFF;
  buffer[6] = 0xFF;
  buffer[7] = 0xFC;
  sendFrameEXTId(controller_id, buffer, 8);
}

void exit_motor_mode_servo(uint8_t controller_id)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFF;
  buffer[3] = 0xFF;
  buffer[4] = 0xFF;
  buffer[5] = 0xFF;
  buffer[6] = 0xFF;
  buffer[7] = 0xFD;
  sendFrameEXTId(controller_id, buffer, 8);
}

// Reference: Data Sheet
// These functions append an integer value to a buffer in big-endian order.
// They take as input a pointer to the buffer, the integer to append, and a pointer to the index where the integer should be inserted.
// After appending, the index is updated to point to the next position in the buffer.
void buffer_append_int32(uint8_t *buffer, int32_t number, uint8_t *index)
{
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t *buffer, int16_t number, uint8_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// These functions send a CAN frame with a specific command to a specified controller.
// Each function constructs a command message by appending the desired value to a buffer, then sends it using the sendFrameEXTId function.
// The controller_id input determines which controller the command is sent to.
void comm_can_set_duty(uint8_t controller_id, float duty)
{
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current)
{ // The range -60000 ~ 60000 represents -60 ~ 60A
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_cb(uint8_t controller_id, float current)
{
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{ // The range -100000 ~ 100000 represents -100000 ~ 100000 electrical speed
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{ // The range-360000000 ~ 360000000 represents position -36000° ~ 36000°
  uint8_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode)
{
  uint8_t buffer[1];
  buffer[0] = set_origin_mode;
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, 1);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA)
{
  uint8_t send_index = 0;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, spd * 10, &send_index);
  buffer_append_int16(buffer, RPA * 10, &send_index);
  sendFrameEXTId(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

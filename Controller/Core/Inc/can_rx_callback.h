#ifndef INC_CAN_RX_CALLBACK_H_
#define INC_CAN_RX_CALLBACK_H_

#include "can.h"
#include "globals.h"

void can_rx_callback(CAN_HandleTypeDef *hcan, IMUData *imuDataRight, IMUData *imuDataLeft, MotorData *motorDataRight, MotorData *motorDataLeft);

#endif /* INC_CAN_RX_CALLBACK_H_ */

#ifndef INC_CAN_RX_CALLBACK_H_
#define INC_CAN_RX_CALLBACK_H_

#include "can.h"
#include "globals.h"
#include "fault_monitor.h"

void can_rx_callback(CAN_HandleTypeDef *hcan, IMUData *imuDataRight, IMUData *imuDataLeft, MotorData *motorDataRight, MotorData *motorDataLeft, FaultMonitor *faults);

#endif /* INC_CAN_RX_CALLBACK_H_ */

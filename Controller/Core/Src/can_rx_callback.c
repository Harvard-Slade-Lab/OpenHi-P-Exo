#include <string.h>
#include "can.h"

#include "can_rx_callback.h"
#include "servo_receiving.h"
#include "imu_processing.h"

int8_t motorDataRightError;
int8_t motorDataLeftError;

CAN_RxHeaderTypeDef Rx0Header;
uint8_t Rx0Data[8];

void can_rx_callback(CAN_HandleTypeDef *hcan, IMUData *imuDataRight, IMUData *imuDataLeft, MotorData *motorDataRight, MotorData *motorDataLeft)
{

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx0Header, Rx0Data) == HAL_OK)
    {
        if (hcan->Instance == CAN2 && Rx0Header.IDE == CAN_ID_EXT)
        {
            if (Rx0Header.ExtId == 0x2900 + motor_ids[0])
            { // Right Motor
                motor_receive(Rx0Data);
                motorDataRight->Position = motor_pos;
                motorDataRight->Speed = motor_spd;
                motorDataRight->Current = motor_cur;
                motorDataRight->Temp = motor_temp;
                motorDataRightError = motor_error;
            }
            else if (Rx0Header.ExtId == 0x2900 + motor_ids[1])
            { // Left Motor
                motor_receive(Rx0Data);
                motorDataLeft->Position = motor_pos;
                motorDataLeft->Speed = motor_spd;
                motorDataLeft->Current = motor_cur;
                motorDataLeft->Temp = motor_temp;
                motorDataLeftError = motor_error;
            }
        }
        else if (hcan->Instance == CAN1 && Rx0Header.IDE == CAN_ID_STD)
        {
            if (Rx0Header.StdId >= IMU_LEFT_ID && Rx0Header.StdId < (IMU_LEFT_ID + 0x100))
            {
                process_imu_data(imuDataLeft, Rx0Header.StdId, Rx0Data);
            }
            else if (Rx0Header.StdId >= IMU_RIGHT_ID && Rx0Header.StdId < (IMU_RIGHT_ID + 0x100))
            {
                process_imu_data(imuDataRight, Rx0Header.StdId, Rx0Data);
            }
        }
        
    }
}

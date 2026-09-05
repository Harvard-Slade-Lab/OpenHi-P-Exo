/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "globals.h"
#include "servo_sending.h"
#include "servo_receiving.h"
#include "sd_card.h"
#include "torque_profile.h"
#include "torque_calculation.h"
#include "can_rx_callback.h"
#include "control_logic.h"
#include "motor_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Tx Data (MCU -> SD Card; SDIO)
TxData dataToSave = {0};
uint8_t paddedTxData[sizeof(TxData) + 4];

// Rx Data (PC -> MCU; BLE)
uint8_t rxDataBLE[RX_BYTES];

/* Torque Profile -----------------------------------------------*/
TorqueProfile profile;

/* Clock and Timer -----------------------------------------------*/
float timeSec = 0.0f;
uint16_t initCnt = 0;

const float initializationTime = 2.0f;
const float calibrationTime = 3.0f;

const uint16_t systemFrequency = 1000; // [Hz]
const uint16_t imuFrequency = 100;	   // [Hz]
const uint16_t motorFrequency = 500;   // [Hz]

/* Sensor Data -----------------------------------------------*/
float loadRight = 0;
float loadLeft = 0;

float voltageSumLeft = 0.0f;
float voltageSumRight = 0.0f;

float imuAngleSumLeft = 0.0f;
float imuAngleSumRight = 0.0f;

float voltageMeanLeft = 0.0f;
float voltageMeanRight = 0.0f;

float imuMeanLeft = 0.0f;
float imuMeanRight = 0.0f;

/* Current Control - AK10-9 -----------------------------------------------*/
float G = 9.0f;
float k_t = 0.12f;
float k_p = 2.0f;
float k_d = 1.0f;

/* Control State Machine -----------------------------------------------*/
uint8_t controlMode = 0; // 0: mode off, 1: mode on;

bool walkingFlagLeft = false;
bool walkingFlagRight = false;

bool eStop = false;
bool motorConnected = true;

const float motorCurLimit = 60.0f; // [A]
const float torqueUpperLimit = 27.0f;
const float torqueLowerLimit = -48.0f;
const float temperatureLimit = 55.0f;

bool newBLEDataFlag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Rx Data (PC -> MCU; BLE)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		newBLEDataFlag = true;
	}
}

void process_ble_data(void)
{
    // frame check
    if (rxDataBLE[0] != 0xAA || rxDataBLE[1] != 0xAA) return;
    if (rxDataBLE[RX_BYTES-2] != 0xBB || rxDataBLE[RX_BYTES-1] != 0xBB) return;

    float params[6];
    memcpy(params, &rxDataBLE[2], sizeof(params));

    torque_profile_set_target(params);
    torque_profile_update_trigger(&profile, timeSec);
}

// Rx Data (Motor -> MCU; CAN)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_rx_callback(hcan, &dataToSave.rightLeg.imu, &dataToSave.leftLeg.imu,
					&dataToSave.rightLeg.motor, &dataToSave.leftLeg.motor);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)
	{ // 1 kHz
		dataToSave.timeMsec += 1.0f;
		timeSec = dataToSave.timeMsec / 1000;

		loadLeft = dataToSave.leftLeg.imu.load;
		loadRight = dataToSave.rightLeg.imu.load;

		get_torques(loadLeft, loadRight,
					voltageMeanLeft, voltageMeanRight,
					&dataToSave.leftLeg.torqueMeasured, &dataToSave.rightLeg.torqueMeasured);

		if (timeSec > initializationTime && timeSec <= initializationTime + calibrationTime)
		{ // After initialization, Start Calibration

			if ((int)dataToSave.timeMsec % (systemFrequency / imuFrequency) == 0)
			{ // 100 Hz
				initCnt++;

				imuAngleSumLeft += dataToSave.leftLeg.imu.roll;
				imuAngleSumRight += dataToSave.rightLeg.imu.roll;

				voltageSumLeft += loadLeft * 3.3f / 4095;
				voltageSumRight += loadRight * 3.3f / 4095;
			}

			if (initCnt == imuFrequency * (int)calibrationTime)
			{
				imuMeanLeft = imuAngleSumLeft / initCnt;
				imuMeanRight = imuAngleSumRight / initCnt;

				voltageMeanLeft = voltageSumLeft / initCnt;
				voltageMeanRight = voltageSumRight / initCnt;

				initCnt = 0;
				imuAngleSumLeft = 0.0f;
				imuAngleSumRight = 0.0f;
				voltageSumLeft = 0.0f;
				voltageSumRight = 0.0f;
			}
		}

		if (timeSec > initializationTime + calibrationTime)
		{ // After Calibration, Start Control
			if ((int)dataToSave.timeMsec % (systemFrequency / motorFrequency) == 0)
			{ // 500 Hz

				if (fabsf(dataToSave.leftLeg.motor.Current) > motorCurLimit || fabsf(dataToSave.rightLeg.motor.Current) > motorCurLimit || dataToSave.leftLeg.torqueMeasured <= torqueLowerLimit || dataToSave.leftLeg.torqueMeasured >= torqueUpperLimit || dataToSave.rightLeg.torqueMeasured <= torqueLowerLimit || dataToSave.rightLeg.torqueMeasured >= torqueUpperLimit || dataToSave.leftLeg.motor.Temp >= temperatureLimit || dataToSave.rightLeg.motor.Temp >= temperatureLimit)
				{
					eStop = true;

					comm_can_set_current(motor_ids[0], 0);
					comm_can_set_current(motor_ids[1], 0);
				}

				if (!eStop && motorConnected)
				{
					if ((controlMode == 1) && ((walkingFlagLeft && walkingFlagRight)))
					{

						motor_current_control(motor_ids[1],
											  dataToSave.leftLeg.torqueDesired,
											  dataToSave.leftLeg.torqueMeasured,
											  dataToSave.leftLeg.motor.Speed,
											  true);

						motor_current_control(motor_ids[0],
											  dataToSave.rightLeg.torqueDesired,
											  dataToSave.rightLeg.torqueMeasured,
											  dataToSave.rightLeg.motor.Speed,
											  false);
					}
					else
					{
						comm_can_set_current(motor_ids[0], 0);
						comm_can_set_current(motor_ids[1], 0);
					}
				}
			}

			if ((int)dataToSave.timeMsec % (systemFrequency / imuFrequency) == 0)
			{ // 100 Hz

				if (newBLEDataFlag)
				{
					newBLEDataFlag = false;
					process_ble_data();			
				}
				
				torque_profile_update_continuous(&profile, timeSec);

				main_controller(timeSec,
								imuMeanLeft, imuMeanRight,
								&dataToSave.leftLeg.imu, &dataToSave.rightLeg.imu,
								&dataToSave.leftLeg.gaitCycleNorm, &dataToSave.rightLeg.gaitCycleNorm,
								&walkingFlagLeft, &walkingFlagRight,
								&profile,
								&dataToSave.leftLeg.torqueDesired, &dataToSave.rightLeg.torqueDesired);

				// Data Save
				memcpy(&paddedTxData[2], &dataToSave, sizeof(TxData));

				// Transmit Data to SD Card via SDIO (MCU)
				if (logState == 2)
				{
					write_data_to_sd((uint8_t *)&paddedTxData);
				}
			}
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_UART_Receive_DMA(&huart1, rxDataBLE, RX_BYTES);

	HAL_TIM_Base_Start_IT(&htim4);

	if (motorConnected)
	{
		for (int i = 0; i < MOTOR_COUNT; i++)
		{
			enter_motor_mode_servo(motor_ids[i]);
			comm_can_set_origin(motor_ids[i], 0);
		}
	}

	torque_profile_init(&profile);

	paddedTxData[0] = 0xAA;
	paddedTxData[1] = 0xAA;
	paddedTxData[sizeof(paddedTxData) - 2] = 0xBB;
	paddedTxData[sizeof(paddedTxData) - 1] = 0xBB;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		sd_card_task();
		update_control_mode(&controlMode);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

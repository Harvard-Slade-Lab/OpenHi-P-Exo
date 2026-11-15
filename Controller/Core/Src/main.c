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
#include "ble_protocol.h"
#include "fault_monitor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLE_ACK_TIMEOUT_MS 5       // ACK frame: 10 bytes, 0.9 ms at 115200 baud
#define BLE_PARTIAL_TIMEOUT_MS 200 // incomplete frame with no new byte for this long: treated as lost
#define BLE_RX_RESTART_MS 10       // reception stopped this long without a restart: restart it
#define CAN_ERROR_NOTIFICATIONS (CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR)

// Independent watchdog: LSI / 128, reload 1249 -> (1249 + 1) * 128 / 32 kHz = 5.0 s (3.4-9.4 s over the
// 17-47 kHz LSI range). The longest main-loop pass is an SD card access: a block write may keep the card
// busy for up to 250 ms (500 ms for SDXC), one f_write can need a few, and card initialization up to 1 s.
// BLE frames (STOP, PARAMS) and the control-mode switch are handled in the same loop: they wait for it.
// A card that stops responding (sd_diskio.c waits up to 30 s) resets the controller, which then keeps
// the motors off (FAULT_WATCHDOG_RESET).
#define IWDG_PRESCALER_DIV128 5U
#define IWDG_RELOAD 1249U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Tx Data (MCU -> SD Card; SDIO)
TxData dataToSave = {0};
uint8_t paddedTxData[sizeof(TxData) + 4];

// Rx Data (PC -> MCU; BLE): circular DMA into the ring, parsed in the main loop
uint8_t bleRxRing[BLE_RX_RING_SIZE];
uint16_t bleRxTail = 0;
uint32_t bleLastByteTick = 0;
volatile uint32_t bleRxStarts = 0; // counts (re)starts of the reception; each starts at ring[0]
uint32_t bleRxStartsSeen = 0;
uint32_t bleRxStoppedTick = 0;
bool bleRxStopped = false;
BleParser bleParser;
BleSession bleSession;

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

volatile uint32_t controlTicks = 0; // TIM4 interrupts; the main loop refreshes the watchdog only while they run

bool eStop = false; // latched: zero current until reset (cause in faultMonitor.faults)
bool motorConnected = true;
FaultMonitor faultMonitor;

const float motorCurLimit = 60.0f; // [A]
const float torqueUpperLimit = 27.0f;
const float torqueLowerLimit = -48.0f;
const float temperatureLimit = 55.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Independent watchdog (no HAL IWDG driver in this project: registers of the CMSIS device header)
static void watchdog_start(void)
{
	__HAL_DBGMCU_FREEZE_IWDG(); // stopped while a debugger halts the core
	IWDG->KR = 0xCCCC; // start (also starts the LSI)
	IWDG->KR = 0x5555; // unlock PR and RLR
	IWDG->PR = IWDG_PRESCALER_DIV128;
	IWDG->RLR = IWDG_RELOAD;
	uint32_t start = HAL_GetTick();
	while (IWDG->SR != 0U && HAL_GetTick() - start < 50U)
	{ // PR and RLR take a few LSI periods to update
	}
	IWDG->KR = 0xAAAA; // reload
}

// Main loop: refresh the watchdog only if the control loop (TIM4) has run since the last refresh
static void watchdog_refresh(void)
{
	static uint32_t controlTicksSeen = 0;
	uint32_t ticks = controlTicks;
	if (ticks != controlTicksSeen)
	{
		controlTicksSeen = ticks;
		IWDG->KR = 0xAAAA;
	}
}

// Rx Data (PC -> MCU; BLE): start the circular DMA reception into the ring
static void ble_rx_start(void)
{
	if (HAL_UART_Receive_DMA(&huart1, bleRxRing, BLE_RX_RING_SIZE) == HAL_OK)
	{
		bleRxStarts++;
	}
}

// A USART1 error (overrun, noise, framing) stops the RX DMA in the HAL: clear it and restart
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		__HAL_UART_CLEAR_PEFLAG(huart); // reads SR then DR: clears PE, FE, NE and ORE
		if (huart->RxState == HAL_UART_STATE_READY)
		{
			ble_rx_start();
		}
	}
}

// Rx Data (PC -> MCU; BLE): apply one frame and ACK it over USART1 TX
static void execute_ble_frame(const BleFrame *frame)
{
	float params[BLE_PARAM_COUNT];
	uint8_t status;
	uint8_t ack[BLE_ACK_FRAME];

	// a latched fault (eStop, or a stop fault the control loop will latch) keeps assistance off
	int faulted = eStop || (faultMonitor.faults & FAULT_STOP_MASK) != 0;
	BleCommand cmd = ble_handle_frame(&bleSession, frame, faulted, params, &status);
	if (cmd != BLE_CMD_NONE)
	{
		HAL_NVIC_DisableIRQ(TIM4_IRQn); // the control loop (TIM4) reads the profile
		if (cmd == BLE_CMD_PARAMS)
		{
			torque_profile_set_target(params);
			torque_profile_update_trigger(&profile, timeSec);
		}
		else
		{
			torque_profile_stop_trigger(&profile, timeSec);
		}
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}

	ble_encode_ack(ack, frame->seq, frame->type, status);
	HAL_UART_Transmit(&huart1, ack, BLE_ACK_FRAME, BLE_ACK_TIMEOUT_MS);
}

// Main loop: parse the bytes the DMA has written since the last call
void process_ble_data(void)
{
	BleFrame frame;

	// reception stopped and not restarted by the error callback: restart it here
	if (huart1.RxState == HAL_UART_STATE_READY)
	{
		if (!bleRxStopped)
		{
			bleRxStopped = true;
			bleRxStoppedTick = HAL_GetTick();
		}
		else if (HAL_GetTick() - bleRxStoppedTick >= BLE_RX_RESTART_MS)
		{
			HAL_NVIC_DisableIRQ(USART1_IRQn);
			HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
			HAL_UART_AbortReceive(&huart1);
			ble_rx_start();
			HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
			HAL_NVIC_EnableIRQ(USART1_IRQn);
			bleRxStopped = false;
		}
	}
	else
	{
		bleRxStopped = false;
	}

	// after a (re)start the DMA writes from ring[0]; bytes not yet parsed are dropped
	uint32_t starts = bleRxStarts;
	if (starts != bleRxStartsSeen)
	{
		bleRxStartsSeen = starts;
		bleRxTail = 0;
		ble_parser_reset(&bleParser);
	}

	uint16_t head = BLE_RX_RING_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	if (head >= BLE_RX_RING_SIZE)
	{
		head = 0;
	}
	__DMB(); // read the ring after the DMA position

	if (head != bleRxTail)
	{
		bleLastByteTick = HAL_GetTick();
	}
	while (bleRxTail != head && bleRxStarts == starts)
	{
		ble_parser_push(&bleParser, bleRxRing[bleRxTail]);
		bleRxTail = (bleRxTail + 1) % BLE_RX_RING_SIZE;
		while (ble_parser_next(&bleParser, &frame))
		{
			execute_ble_frame(&frame);
		}
	}

	// a frame missing bytes (lost BLE chunk) must not hold the frames received after it
	if (bleParser.len > 0 && HAL_GetTick() - bleLastByteTick >= BLE_PARTIAL_TIMEOUT_MS)
	{
		while (ble_parser_flush(&bleParser, &frame))
		{
			execute_ble_frame(&frame);
		}
	}
}

// Rx Data (Motor -> MCU; CAN)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_rx_callback(hcan, &dataToSave.rightLeg.imu, &dataToSave.leftLeg.imu,
					&dataToSave.rightLeg.motor, &dataToSave.leftLeg.motor, &faultMonitor);
}

// CAN error warning, error passive or bus-off (the hardware recovers from bus-off by itself)
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	fault_monitor_can_error(&faultMonitor, hcan->Instance == CAN1 ? 1 : 2,
							(hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0U);
	HAL_CAN_ResetError(hcan);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)
	{ // 1 kHz
		controlTicks++;
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
					fault_monitor_set(&faultMonitor, FAULT_LIMIT);
				}
				if (canTxDropCount != 0)
				{ // a motor command was not sent
					fault_monitor_set(&faultMonitor, FAULT_MOTOR_TX);
				}

				bool assisting = (controlMode == 1) && walkingFlagLeft && walkingFlagRight;
				if (fault_monitor_check(&faultMonitor, HAL_GetTick(), loadLeft, loadRight, assisting))
				{
					eStop = true;
				}

				if (eStop)
				{ // every period: a single CAN frame can be lost (no automatic retransmission)
					comm_can_set_current(motor_ids[0], 0);
					comm_can_set_current(motor_ids[1], 0);
				}
				else if (motorConnected)
				{
					if (assisting)
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

				torque_profile_update_continuous(&profile, timeSec);

				main_controller(timeSec,
								imuMeanLeft, imuMeanRight,
								&dataToSave.leftLeg.imu, &dataToSave.rightLeg.imu,
								&dataToSave.leftLeg.gaitCycleNorm, &dataToSave.rightLeg.gaitCycleNorm,
								&walkingFlagLeft, &walkingFlagRight,
								&profile,
								&dataToSave.leftLeg.torqueDesired, &dataToSave.rightLeg.torqueDesired);

				// Data Save
				dataToSave.faultCode = faultMonitor.faults;
				dataToSave.motorErrorRight = faultMonitor.motorError[FAULT_SIDE_RIGHT];
				dataToSave.motorErrorLeft = faultMonitor.motorError[FAULT_SIDE_LEFT];
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

	fault_monitor_init(&faultMonitor, HAL_GetTick()); // before the CAN interrupts use it
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{ // the watchdog reset the controller: keep the motors off until the next reset
		fault_monitor_set(&faultMonitor, FAULT_WATCHDOG_RESET);
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();

	// CAN errors reach HAL_CAN_ErrorCallback through the SCE interrupts (handlers in stm32f4xx_it.c)
	HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
	HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_ERROR_NOTIFICATIONS);

	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_ERROR_NOTIFICATIONS);

	ble_parser_reset(&bleParser);
	ble_session_reset(&bleSession);
	ble_rx_start();

	HAL_TIM_Base_Start_IT(&htim4);

	if (motorConnected)
	{
		for (int i = 0; i < MOTOR_COUNT; i++)
		{
			enter_motor_mode_servo(motor_ids[i]);
			comm_can_set_origin(motor_ids[i], 0);
			comm_can_set_current(motor_ids[i], 0); // after a watchdog reset the last command may still be active
		}
	}

	torque_profile_init(&profile);

	paddedTxData[0] = 0xAA;
	paddedTxData[1] = 0xAA;
	paddedTxData[sizeof(paddedTxData) - 2] = 0xBB;
	paddedTxData[sizeof(paddedTxData) - 1] = 0xBB;

	watchdog_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		watchdog_refresh();
		sd_card_task();
		update_control_mode(&controlMode);
		process_ble_data();
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

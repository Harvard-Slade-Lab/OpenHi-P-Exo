#include <stdio.h>
#include <string.h>
#include "fatfs.h"

#include "globals.h"
#include "sd_card.h"

// Definitions and Variables
node buffer[NODE_LEN];
node *head;
uint8_t sd_buffer[DATA_BLK_SIZE];
UINT bc;

uint8_t sdState = 1; // 0: Initialized, 1: Not yet
uint8_t logState = 0; // 0: Idle, 1: File generation, 2: Data logging, 3: Logging stop

// SD Card Data Logging File Name
FRESULT resFile;
char fileName[] = "hip";
int iSDFile = 1;
char newFileName[20];

void init_node() {
	int i;
	for (i = 0; i < NODE_LEN; i++) {
		buffer[i].count = 0;
		buffer[i].next = NULL;
	}

	head = &buffer[0];
}

void write_data_to_sd(uint8_t *data) {
	node *p;
	p = head;

	// [[ 1. Find last node ]]
	while (p->next != NULL) {
		p = p->next;
	}
	if (p->count == DATA_CNT)
		return;

	// [[ 2. Write data ]]
	memcpy(p->data + p->count * (sizeof(TxData) + 4), data, sizeof(TxData) + 4);
	p->count++;

	// [[ 3. If node is full -> link next node ]]
	if (p->count == DATA_CNT) {
		int i;
		for (i = 0; i < NODE_LEN; i++) {
			if (buffer[i].count == 0)
				break;
		}

		if (i == NODE_LEN)
			return;
		p->next = &buffer[i];
	}
}

uint8_t get_data_from_sd(uint8_t *data) {
	node *p;
	p = head;

	if (head->count == DATA_CNT) {
		memcpy(data, head->data, DATA_BLK_SIZE);
		head->count = 0;
		head = head->next;
		p->next = NULL;

		return 1;
	}

	return 0;
}

void sd_card_task(void) {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) { // SDIO_CD - SD card is inserted
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // SDIO_CD "LED OFF"
		if (sdState == 0) { // SD Card was already initialized but your SD Card was removed
			sdState = 1; // You need to Initialize the SD Card Again
		}
	} else { // SD card is removed
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // SDIO_CD "LED ON"
		if (sdState != 0) { // SD Card has not been initialized yet
			sdState = BSP_SD_Init();
		}
	}

	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) { // DATA_LOG - Switch is off
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // DATA_LOG "LED OFF"
		if (logState == 2) { // Your data is logging to SD Card now
			logState = 3; // Finish data logging to SD Card
		}
	} else { // Switch is on
		if (sdState == 0 && logState == 0) { // SD Card was initialized
			logState = 1; // Start logging data to SD Card
		}

		if (logState == 2) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // DATA_LOG "LED ON"	
		} else { // Logging is stopped for some reasons 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // DATA_LOG "LED OFF"
		}
	}

	switch (logState) {
	case 0: // Idle
		break;

	case 1: // File generation

		if (sdState == 0) { // SD Card was successfully initialized
			sprintf(newFileName, "%s%03d.bin", fileName, iSDFile);

			if ((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) == FR_OK) { // Mounting SD Card Success
				resFile = f_open(&SDFile, newFileName, FA_READ);
				if (resFile != FR_OK) { // File does not exist, create a new one
					resFile = f_open(&SDFile, newFileName,
						FA_CREATE_ALWAYS | FA_WRITE);
					if (resFile == FR_OK) { // File created successfully
						init_node();
						logState = 2; // Now you can go for logging
					} else
						logState = 0; // File creating failed
				} else { // File exists, create a new one with new file name
					f_close(&SDFile);
					iSDFile++;
				}
			} else
				logState = 0; // Mount Failed

			break;
		}

	case 2: // Data logging

		if (get_data_from_sd(sd_buffer)) {
			if (f_write(&SDFile, sd_buffer, DATA_BLK_SIZE, &bc) != FR_OK) {
				logState = 0;
			}
		}
		break;

	case 3: // Logging stop

		f_close(&SDFile);
		logState = 0;

		break;
	}
}

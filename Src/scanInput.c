/*
 * scanInput.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */

#include "globals.h"
#include "structures.h"

#include "scanInput.h"
#include "controllerUtils.h"

extern UART_HandleTypeDef huart1;
extern enum IndicationState indicationState;

void xScanInputTask(void* arguments){
	uint8_t prevInputState = 1;
	uint8_t curInputState = 1;

	for(;;){
		curInputState = HAL_GPIO_ReadPin(HALL_SENS_PORT, HALL_SENS_PIN);
		if (curInputState != prevInputState){
			vTaskDelay(1 / portTICK_RATE_MS);
			curInputState = HAL_GPIO_ReadPin(HALL_SENS_PORT, HALL_SENS_PIN);
			if (curInputState != prevInputState){
				prevInputState = curInputState;
				if (!prevInputState){
					//Change indication to searching
					//HAL_GPIO_WritePin(A_LED_PORT, A_LED_PIN, GPIO_PIN_SET);
					indicationState = SEARCH;
					//Change channel
					//rf433_set_channel_1();
					CMD_RF_ON;
					vTaskDelay(50 / portTICK_RATE_MS);

					HAL_UART_Transmit(&huart1, (uint8_t*) "AT+C001\r", 8, 0x2000);

					vTaskDelay(50 / portTICK_RATE_MS);
					CMD_RF_OFF;
				}
			}
		}

		vTaskDelay(10 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}


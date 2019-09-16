/*
 * controllerUtils.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */

#include "globals.h"
#include "structures.h"

extern UART_HandleTypeDef huart1;

char debugMessage[64] = {};
uint8_t debugMessageLength = 0;

void print_debug(char* msg){
	#if DEBUG_SERIAL
		debugMessageLength = sprintf(debugMessage, "%s", msg);
		HAL_UART_Transmit(&huart1, (uint8_t*) debugMessage, debugMessageLength, 0x2000);
	#endif
}

void init_rf433(uint8_t channel){
	CMD_RF_ON;
	HAL_Delay(50);

	HAL_UART_Transmit(&huart1, (uint8_t*) "AT+FU1\r", 7, 0x2000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1, (uint8_t*) "AT+B19200\r", 7, 0x2000);
	HAL_Delay(200);
	debugMessageLength = sprintf(debugMessage, "AT+C%03d\r", channel);
	HAL_UART_Transmit(&huart1, (uint8_t*) debugMessage, debugMessageLength, 0x2000);
	HAL_Delay(200);

	CMD_RF_OFF;
	HAL_Delay(50);

	huart1.Init.BaudRate = 19200;
	if (HAL_UART_Init(&huart1) != HAL_OK){
		Error_Handler();
	}
}

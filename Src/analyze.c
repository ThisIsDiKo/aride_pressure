/*
 * analyze.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */

#include "globals.h"
#include "structures.h"
#include "analyze.h"

#include "math.h"

extern uint16_t nessPressure[4];
extern uint16_t filteredData[4];

extern enum WorkState workState;
extern enum Compensation pressureCompensation;

extern uint8_t pressIsLower[4];

extern xSemaphoreHandle xPressureCompensationSemaphore;
extern UART_HandleTypeDef huart1;

uint8_t analyzeCounter[4] = {0};
extern enum AirSystemType airSystem;

extern GPIO_TypeDef *UP_PORT[4];
extern uint32_t UP_PIN[4];
extern GPIO_TypeDef *DOWN_PORT[4];
extern uint32_t DOWN_PIN[4];

void xAnalyzeTask(void *arguments){
	portBASE_TYPE xStatus;
	uint8_t i = 0;

	int16_t deltaPressure = 0;
	uint8_t prescalerCounter = 10;
	//uint8_t analyzeCounterRef[4] = {5};
	int16_t pressureThreshold = 10;

	xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
	for(;;){
		xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
		if (xStatus == pdPASS){
			if (airSystem == RECEIVER){
				workState = FREE;
				for (i = 0; i < 4; i++){
					if (pressIsLower[i]){
						deltaPressure = nessPressure[i] - filteredData[i];
						deltaPressure = abs(deltaPressure);
						if (deltaPressure > pressureThreshold){
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
							workState = WORKING;
						}
					}
					else {
						deltaPressure = filteredData[i] - nessPressure[i];
						deltaPressure = abs(deltaPressure);
						if (deltaPressure > pressureThreshold){
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
							workState = WORKING;
						}
					}
				}

				if (workState == FREE){
					pressureCompensation = OFF;
				}
				else{
					pressureCompensation = ON;
				}

				vTaskDelay(500 / portTICK_RATE_MS);
				for (i = 0; i < 4; i++){
					HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
				}
				vTaskDelay(500 / portTICK_RATE_MS);
				for (i = 0; i < 4; i++){
					HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
				}

				vTaskDelay(1000 / portTICK_RATE_MS);
			}
			else{
				if (prescalerCounter >= 5){
					prescalerCounter = 0;
					workState = FREE;

					for (i = 0; i < 4; i++){
						if (analyzeCounter[i] < 5){
							workState = WORKING;
							if (pressIsLower[i]){
								if (filteredData[i] < nessPressure[i]){
									analyzeCounter[i] = 0;
									HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
									HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
								}
								else{
									analyzeCounter[i] += 1;
									HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
									HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
								}
							}
							else{
								if (filteredData[i] > nessPressure[i]){
									analyzeCounter[i] = 0;
									HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
									HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
								}
								else{
									analyzeCounter[i] += 1;
									HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
									HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
								}
							}
						}
						else{
							analyzeCounter[i] = 0;
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
						}
					}

					if (workState == FREE){
						prescalerCounter = 10;
						pressureCompensation = OFF;
					}
					else{
						pressureCompensation = ON;
					}
				}
				else{
					prescalerCounter++;
				}
			}
		}
	}
}

/*
 * analyze.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */

#include "globals.h"
#include "structures.h"
#include "analyze.h"
#include "flashFunctions.h"

#include "math.h"

extern uint16_t nessPressure[4];
extern uint16_t filteredData[4];

extern enum WorkState workState;
extern enum Compensation pressureCompensation;

extern int8_t pressIsLower[4];

extern xSemaphoreHandle xPressureCompensationSemaphore;
extern UART_HandleTypeDef huart1;

uint8_t analyzeCounter[4] = {0};
extern enum AirSystemType airSystem;

extern GPIO_TypeDef *UP_PORT[4];
extern uint32_t UP_PIN[4];
extern GPIO_TypeDef *DOWN_PORT[4];
extern uint32_t DOWN_PIN[4];

extern char message[128];
extern uint8_t messageLength;
extern UART_HandleTypeDef huart1;
extern struct controllerData controllerSettings;

void xAnalyzeTask(void *arguments){
	portBASE_TYPE xStatus;
	uint8_t i = 0;

	int16_t deltaPressure = 0;
	uint8_t prescalerCounter = 10;
	//uint8_t analyzeCounterRef[4] = {5};
	int16_t pressureThreshold = 40;

	uint16_t medArray[4][7];
	uint8_t counter = 0;
	uint8_t smallestIndex = 0;
	uint8_t currentIndex = 0;
	uint16_t temp = 0;

	uint16_t startPressure[4];
	uint32_t impCounter = 0;
	uint32_t dCounter = 0;
	uint8_t stopImp = 0;
	volatile int32_t impTime[4] = {0, 0, 0, 0};
	float impCoeff[4] = {0.0,0.0,0.0,0.0};
	uint16_t calibrationImp = 1000;

	xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
	for(;;){
		xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
		if (xStatus == pdPASS){
			if (airSystem == RECEIVER){
				workState = FREE;

				#if DEBUG_SERIAL
					messageLength = sprintf(message, "+++NEW STEP+++\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
				#endif

				//Look at pressure
				for (i = 0; i < 4; i++){
					startPressure[i] = filteredData[i];
					deltaPressure = nessPressure[i] - filteredData[i];
					deltaPressure = abs(deltaPressure);

					if (deltaPressure > pressureThreshold){
						if (nessPressure[i] > filteredData[i])
							pressIsLower[i] = 1;
						else
							pressIsLower[i] = 0;
						workState = WORKING;
					}
					else{
						pressIsLower[i] = -1;
					}
				}

				//finish compensation
				if (workState == FREE){
					pressureCompensation = OFF;
					impTime[0] = 0;
					impTime[1] = 0;
					impTime[2] = 0;
					impTime[3] = 0;
					continue;
				}
				else{
					pressureCompensation = ON;
				}

				//calculate impulse
				#if DEBUG_SERIAL
					messageLength = sprintf(message, "---IMP DATA---\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
				#endif

				for (i = 0; i < 4; i++){
					deltaPressure = nessPressure[i] - filteredData[i];
					if (pressIsLower[i] == 1){

						impTime[i] = (int32_t)(controllerSettings.impUpCoeff[i] * (float)deltaPressure);
#if DEBUG_SERIAL
	messageLength = sprintf(message, "%d: up %ld\n", i, impTime[i]);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif
						if (impTime[i] < 0) impTime[i] = 0;
						else if (impTime[i] == 0) impTime[i] = 1000;
						else if (impTime[i] > 30000) impTime[i] = 1000;
					}
					else if (pressIsLower[i] == 0){
						impTime[i] = (int32_t)(controllerSettings.impDownCoeff[i] * (float)deltaPressure);
#if DEBUG_SERIAL
	messageLength = sprintf(message, "%d: down %ld\n", i,  impTime[i]);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif
						if (impTime[i] < 0) impTime[i] = 0;
						else if (impTime[i] == 0) impTime[i] = 500;
						else if (impTime[i] > 30000) impTime[i] = 500;
					}
					else{
						impTime[i] = 0;
					}
#if DEBUG_SERIAL
	messageLength = sprintf(message, "%d: %ld\n", i, impTime[i]);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif


				}

				vTaskDelay(500);
				impCounter = 0;

				if (pressureCompensation == OFF){
					for (i = 0; i < 4; i++){
						HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
						HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
					}
					impTime[0] = 0;
					impTime[1] = 0;
					impTime[2] = 0;
					impTime[3] = 0;
					continue;
				}

				for (i = 0 ; i < 4; i++){
					if (impTime[i] > 0){
						if (pressIsLower[i] == 1){
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
						}
						else if (pressIsLower[i] == 0){
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
						}
					}
				}

				impCounter = xTaskGetTickCount();
				while(1){
					stopImp = 0;
					vTaskDelay(100);
					dCounter = xTaskGetTickCount() - impCounter;

					for (i = 0 ; i < 4; i++){
						if(dCounter > impTime[i]){
#if DEBUG_SERIAL
	messageLength = sprintf(message, "%d: off %ld\t %ld\n",i, impTime[i], xTaskGetTickCount() - impCounter);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
							stopImp += 1;
						}
						else{
							if (pressIsLower[i] == 1){
								HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
							}
							else if (pressIsLower[i] == 0){
								HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
							}
						}
					}
					if (stopImp >= 4){
						break;
					}
				}
#if DEBUG_SERIAL
	messageLength = sprintf(message, "all imp off after %ld\n", xTaskGetTickCount() - impCounter);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif
				vTaskDelay(1000);

				for (i = 0 ; i < 4; i++){
					if (pressIsLower[i] >=0){
						deltaPressure = filteredData[i] - startPressure[i];
						impCoeff[i] = (float)impTime[i] / (float) deltaPressure;
						if (pressIsLower[i] == 1){
							controllerSettings.impUpCoeff[i] = impCoeff[i];
						}
						else if (pressIsLower[i] == 0){
							controllerSettings.impDownCoeff[i] = impCoeff[i];
						}
#if DEBUG_SERIAL
	messageLength = sprintf(message, "%d: %d\t%d\t%d\t%ld\t%d\t%d\n", i, nessPressure[i], startPressure[i], filteredData[i], impTime[i],(int)controllerSettings.impUpCoeff[i],(int)controllerSettings.impDownCoeff[i]);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageLength, 0xFFFF);
#endif
					}
				}
				mWrite_flash();
			}
			else{

			}
		}
	}
}
/*
 * analyze.c
 *
 *  Created on: 4 окт. 2019 г.
 *      Author: ADiKo
 */



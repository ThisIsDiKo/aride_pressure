/*
 * analyze.c
 *
 *  Created on: 16 ����. 2019 �.
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

extern int8_t pressIsLower[4];

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
	int16_t pressureThreshold = 20;

	uint16_t medArray[4][7];
	uint8_t counter = 0;
	uint8_t smallestIndex = 0;
	uint8_t currentIndex = 0;
	uint16_t temp = 0;

	uint16_t startPressure[4];
	uint32_t impCounter = 0;
	uint8_t stopImp = 0;
	xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
	for(;;){
		xStatus = xSemaphoreTake(xPressureCompensationSemaphore, portMAX_DELAY);
		if (xStatus == pdPASS){
			if (airSystem == RECEIVER){
				workState = FREE;


				for (i = 0; i < 4; i++){
					medArray[i][counter] = filteredData[i];
				}

				if (counter < 7){
					counter += 1;
					continue;
				}

				counter = 0;

				for (i = 0; i < 4; i++){
					for (counter = 0; counter < 6; ++counter){
						smallestIndex  = counter;
						for (currentIndex = counter + 1; currentIndex < 7; ++currentIndex)
							if (medArray[i][currentIndex])
								smallestIndex = currentIndex;

						temp = medArray[i][counter];
						medArray[i][counter] = medArray[i][smallestIndex];
						medArray[i][counter] = temp;
					}
				}
				//Got median values

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


				for (i = 0; i < 4; i++){
					if (pressIsLower[i] == 0){
						HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
					}
					else if (pressureIsLower[i] == 1){
						HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
					}
					else{
						continue; // do nothing
					}

					vTaskDelay(1000);

					HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);

					vTaskDelay(1000);
				}

				//perehod
				if (workState == FREE){
					pressureCompensation = OFF;
					continue;
				}
				else{
					pressureCompensation = ON;
				}

				vTaskDelay(2000);

				for (i = 0 ; i < 4; i++){
					deltaPressure = filteredData[i] - startPressure[i];
					impCoeff[i] = (float) 1000 / (float) deltaPressure;
					impTime[i] = (int16_t)(impCoeff[i] * (float)(nessPressure[i] - filteredData[i]));
					if ((impTime[i] < 0) || (impTime[i] > 60000)){
						impTime[i] = 0;
					}
				}

				impCounter = 0;

				for (i = 0 ; i < 4; i++){
					if (impTime[i] > 0){
						if (pressIsLower[i] == 0){
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_SET);
						}
						else if (pressIsLower[i] == 1){
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_SET);
						}
					}
				}

				while(1){
					stopImp = 0;
					vTaskDelay(5);
					impCounter += 5;

					for (i = 0 ; i < 4; i++){
						if(impCounter > impTime[i]){
							HAL_GPIO_WritePin(DOWN_PORT[i], DOWN_PIN[i], GPIO_PIN_RESET);
							HAL_GPIO_WritePin(UP_PORT[i], UP_PIN[i], GPIO_PIN_RESET);
							stopImp = +1;
						}
					}
					if (stopImp = 4){
						break;
					}
				}

				vTaskDelay(2000);
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
/*
 * analyze.c
 *
 *  Created on: 4 ���. 2019 �.
 *      Author: ADiKo
 */



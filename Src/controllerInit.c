/*
 * controllerInit.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */


#include "globals.h"

#include "controllerInit.h"

#include "flashFunctions.h"
#include "adcStorage.h"
#include "processCommand.h"
#include "analyze.h"
#include "structures.h"
#include "scanInput.h"
#include "controllerUtils.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;

extern xSemaphoreHandle xPressureCompensationSemaphore;
extern xQueueHandle xRecCommandQueue;
extern char message[128];
extern uint8_t recCommandByte;

extern struct controllerData controllerSettings;

extern uint16_t server_UID;
uint32_t unique_ID[3] = {0};

enum IndicationState indicationState = NORMAL_NC;

void xBlynkTask(void* arguments){
	for(;;){
		if (indicationState == NORMAL_C){
			TIM3->CCR2 = 10000;
		}
		else if (indicationState == NORMAL_NC){
			TIM3->CCR1 = 10000;
		}
		else if (indicationState == SEARCH){
			TIM3->CCR3 = 10000;
		}
		else if (indicationState == COMPENSATION){
			TIM3->CCR2 = 10000;
			TIM3->CCR3 = 10000;
		}

		vTaskDelay(500 / portTICK_RATE_MS);
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		vTaskDelay(500 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

void controller_init(){
	mRead_flash();
	if(controllerSettings.rfChannel > 120){
		controllerSettings.rfChannel = 1;
	}

	HAL_GetUID(unique_ID);

	server_UID = (unique_ID[0] + unique_ID[1] + unique_ID[2]) / 65536;

	init_rf433(controllerSettings.rfChannel); //TODO: change to more common
//	sprintf(message, "ID: %ld\r\n", server_UID);
//				HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_UART_Receive_IT(&huart1, &recCommandByte, 1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);

	vSemaphoreCreateBinary(xPressureCompensationSemaphore);


	#if DEBUG_SERIAL
		uint32_t fre=xPortGetFreeHeapSize();
		sprintf(message, "Free heap: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif

	xTaskCreate(xBlynkTask,
				"Blynk",
				200,
				NULL,
				1,
				NULL);

	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after Blynk: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif

	xTaskCreate(xScanInputTask,
				"Scan",
				200,
				NULL,
				1,
				NULL);

	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after Scan: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif

	xTaskCreate(xStoreADCDataTask,
				"SADCData",
				512,
				NULL,
				1,
				NULL);

	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after SADCData: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif

	xTaskCreate(xProcessCommandTask,
				"ProcCmd",
				512,
				NULL,
				3,
				NULL);

	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after ProcCmd: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif

	xTaskCreate(xAnalyzeTask,
				"AnTsk",
				400,
				NULL,
				1,
				NULL);
	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after AnTask: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*) message, strlen(message), 0xFFFF);
	#endif

	xRecCommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, MAX_COMMAND_LENGTH);
	//xRecCommandQueue = xQueueCreate(1, 1);

	#if DEBUG_SERIAL
		fre=xPortGetFreeHeapSize();
		sprintf(message, "heap after queue: %ld\r\n", fre);
		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
	#endif
}

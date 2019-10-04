/*
 * processCommand.c
 *
 *  Created on: 16 сент. 2019 г.
 *      Author: ADiKo
 */
#include "globals.h"
#include "structures.h"

#include "processCommand.h"
#include "flashFunctions.h"
#include "controllerUtils.h"

extern UART_HandleTypeDef huart1;

extern uint16_t filteredData[4];
extern struct controllerData controllerSettings;
extern xQueueHandle xRecCommandQueue;

extern char message[128];
extern uint8_t messageLength;
extern uint16_t nessPressure[4];
extern enum Compensation pressureCompensation;
extern enum IndicationState indicationState;

int8_t pressIsLower[4] = {0};
extern uint8_t lastTimeCommand;

extern uint16_t server_UID;
extern enum AirSystemType airSystem;

void xProcessCommandTask(void* arguments){
	portBASE_TYPE xStatus;
	uint8_t command[MAX_COMMAND_LENGTH] = {0};

	uint16_t id = 0;
	uint8_t i = 0;
	char co = 0;
	char outputState = 0;
	uint16_t channel = 0;

	char prevOutputState = 0;
	//uint16_t sendData[4] = {0};
	char systemType = 0;

	for(;;){
		xStatus = xQueueReceive(xRecCommandQueue, command, portMAX_DELAY);
		if (xStatus == pdPASS){

			lastTimeCommand = 0;

			switch(command[0]){

				case 'm':{

					if (indicationState != SEARCH){
						indicationState = NORMAL_C;
					}

					sscanf((char*)command, "m,%hu,%c,%c,\n", &id, &co, &outputState);
					outputState = command[10];
					if (id == server_UID){
						messageLength = sprintf(message, "m,%hu,%hu,%hu,%hu,%hu,\n", 	controllerSettings.clientID,
																						filteredData[SENS_1],
																						filteredData[SENS_2],
																						filteredData[SENS_3],
																						filteredData[SENS_4]);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*) message, messageLength);

						if (outputState != prevOutputState){

							pressureCompensation = OFF;

							if (outputState & 0b00000001) 	C1_UP_ON;
							else 				   			C1_UP_OFF;
							if (outputState & 0b00000010) 	C1_DOWN_ON;
							else 				   			C1_DOWN_OFF;

							if (outputState & 0b00000100) 	C2_UP_ON;
							else 				   			C2_UP_OFF;
							if (outputState & 0b00001000) 	C2_DOWN_ON;
							else 				   			C2_DOWN_OFF;

							if (outputState & 0b00010000) 	C3_UP_ON;
							else 				   			C3_UP_OFF;
							if (outputState & 0b00100000) 	C3_DOWN_ON;
							else 				   			C3_DOWN_OFF;

							if (outputState & 0b01000000) 	C4_UP_ON;
							else 				   			C4_UP_OFF;
							if (outputState & 0b10000000) 	C4_DOWN_ON;
							else 				   			C4_DOWN_OFF;

							prevOutputState = outputState;
						}
					}
					break;
				}
				case 's':{
					if (command[1] == 'x'){
						sscanf((char*)command, "sx,%hu,\n", &id);
						if (id == server_UID){
							pressureCompensation = OFF;
						}
					}
					else if (command[1] == ','){
						sscanf((char*)command, "s,%hu,%hu,%hu,%hu,%hu,%c,\n", &id, 	&nessPressure[0],
																					&nessPressure[1],
																					&nessPressure[2],
																					&nessPressure[3],
																					&systemType);
						if (id == server_UID){
							if (systemType == '1'){ // air system choice
								airSystem = COMPRESSOR; // for compressor
							}
							else{
								airSystem = RECEIVER;
							}
							for (i = 0; i < 4; i++){
								if (filteredData[i] > nessPressure[i])
									pressIsLower[i] = 0;
								else
									pressIsLower[i] = 1;
							}
							pressureCompensation = ON;
						}
					}
					break;
				}
				case 'x':{
					if (indicationState == SEARCH){
						if (command[1] == '?'){
							sscanf((char*)command, "x?%hu,\n", &controllerSettings.clientID);
							messageLength = sprintf(message, "x,%05d,%05d,\n", controllerSettings.clientID, server_UID);
							HAL_UART_Transmit_DMA(&huart1, (uint8_t*) message, messageLength);
						}
						else if (command[1] == 'c'){
							sscanf((char*)command, "xc,%hu,%hu,\n", &id, &channel);

							messageLength = sprintf(message, "id,%05d,%05d,%03d\n", id, server_UID, channel);
							print_debug(message);
							//HAL_UART_Transmit(&huart1, (uint8_t*) message, messageLength, 0x2000);

							if (id == server_UID){
								controllerSettings.rfChannel = channel;
								mWrite_flash();
								messageLength = sprintf(message, "xc,%05d,ok,\n", controllerSettings.clientID);
								HAL_UART_Transmit_DMA(&huart1, (uint8_t*) message, messageLength);

								vTaskDelay(200 / portTICK_RATE_MS);
								CMD_RF_ON;
								vTaskDelay(50 / portTICK_RATE_MS);

								messageLength = sprintf(message, "AT+C%03d\r", channel);
								HAL_UART_Transmit_DMA(&huart1, (uint8_t*) message, messageLength);

								vTaskDelay(50 / portTICK_RATE_MS);
								CMD_RF_OFF;

								indicationState = NORMAL_C;

							}
						}
					}
					break;
				}
			}
		}
	}
	vTaskDelete(NULL);
}

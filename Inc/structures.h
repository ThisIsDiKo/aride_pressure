/*
 * structures.h
 *
 *  Created on: 20 рту. 2019 у.
 *      Author: ADiKo
 */

#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#include "globals.h"

struct controllerData{
	uint8_t  rfChannel;
	uint8_t  rere;
	uint16_t rfBaudRate;
	uint16_t clientID;
	uint16_t qeqe;
	uint32_t rfMaster;
	float impUpCoeff[4];
	float impDownCoeff[4];
};

enum WorkState{
	FREE,
	WORKING
};

enum Compensation{
	OFF,
	ON
};

enum IndicationState{
	NORMAL_C,
	NORMAL_NC,
	SEARCH,
	COMPENSATION
};

enum AirSystemType{
	COMPRESSOR,
	RECEIVER
};

#endif /* STRUCTURES_H_ */

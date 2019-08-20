/*
 * structures.h
 *
 *  Created on: 20 ���. 2019 �.
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
};

enum WorkState{
	FREE,
	WORKING
};

enum Compensation{
	OFF,
	ON
};

#endif /* STRUCTURES_H_ */
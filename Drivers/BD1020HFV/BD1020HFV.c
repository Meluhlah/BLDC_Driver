/*
 * BD1020HFV.c
 *
 *  Created on: Nov 4, 2024
 *      Author: Lidor
 */


#include "BD1020HFV.h"

float get_temperature(uint16_t adcValue){
	float temp;
	if((V30 - adcValue) >= 0)
	{
		temp = (V30 - adcValue) / DELTA_1C;
		return REF_TEMP + temp;
	}
	else
		temp = (adcValue - V30) / DELTA_1C;
	return REF_TEMP - temp;
}



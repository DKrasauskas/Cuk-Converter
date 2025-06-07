/*
 * utils.h
 *
 *  Created on: Jun 7, 2025
 *      Author: domin
 */

#ifndef SRC_CONTROL_UTILS_H_
#define SRC_CONTROL_UTILS_H_
#pragma once
#include "../SPI/typedefs.h"

Data __init_Data__(){
	Data d;
	//I_IN sensor
	S1.address = 0;
	S1.data = 255;
	S1.pin = 12;
	S1.id = 1;
	//V_IN sensor
	S2.address = 0;
	S2.data = 255;
	S2.pin = 12;
	S2.id = 1;
	//V_OUT sensor
	S3.address = 0;
	S3.data = 255;
	S3.pin = 12;
	S3.id = 1;
	//I_OUT sensor
	S4.address = 0;
	S4.data = 255;
	S4.pin = 12;
	S4.id = 1;
	//assign
	d.I_IN  = &S1;
	d.V_IN  = &S2;
	d.V_OUT = &S3;
	d.I_OUT = &S4;
	return d;
}

#endif /* SRC_CONTROL_UTILS_H_ */

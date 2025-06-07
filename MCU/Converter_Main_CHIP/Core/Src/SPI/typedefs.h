/*
 * typedefs.h
 *
 *  Created on: Jun 7, 2025
 *      Author: domin
 */

#ifndef SRC_SPI_TYPEDEFS_H_
#define SRC_SPI_TYPEDEFS_H_

typedef struct{
	uint8_t pin, id;
	uint16_t address, data;
	//float data[3];
}Sensor;

typedef struct{
	Sensor* I_IN, *V_IN, *I_OUT, *V_OUT;
	uint8_t overrun_id;
}Data;

typedef struct{
	 uint16_t* buffer_tx, *buffer_rx;
	 uint8_t busy, id, id_prev;
	 uint8_t CS_PIN;
	 uint8_t len_tx;
	 uint8_t len_rx;
}Message;

typedef struct{
	float V_OUT;
}OutputConfig;

#endif /* SRC_SPI_TYPEDEFS_H_ */

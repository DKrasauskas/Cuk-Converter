/*
 * SPI.h
 *
 *  Created on: Jun 5, 2025
 *      Author: domin
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_
#include "typedefs.h"

Sensor S1, S2, S3, S4;
volatile Message comnet;

//SPI_HANDLE requests data from the sensors


void SPI_INIT_TRANSFER(Sensor* sens){
	if (comnet.id != sens->id || comnet.busy == 1) return; //return if BUS BUSY
	comnet.CS_PIN = sens->pin;
	comnet.id_prev += 1;
	comnet.id += 1;
	if(comnet.id == 3){
			//HAL_Delay(1);
		}
		if(comnet.id == 5){
				HAL_Delay(10);
			}
	GPIOB->BSRR |= ((uint16_t)1 << (comnet.CS_PIN + 16)); //pull CS low
	comnet.buffer_tx = &sens->address;
	comnet.buffer_rx = &sens->data;
	comnet.len_tx = 1;
	comnet.len_rx = 1;
	comnet.busy = 1;


	SPI2->CR2 |= (1 << 7); //ENABLE TEXEIE INTERUPT
	SPI2->CR2 |= (1 << 6); //ENABLE RXNEIE INTERUPT
	SPI2->CR1 |= (1 << 6); //ENABLE SPI2
}

void SPI_HANDLE(Data* data){
	if(!comnet.busy){
    	comnet.id = (comnet.id == data->overrun_id) ? 1 : comnet.id;
    	SPI_INIT_TRANSFER(data->I_IN);
    	SPI_INIT_TRANSFER(data->V_IN);
    	SPI_INIT_TRANSFER(data->I_OUT);
    	SPI_INIT_TRANSFER(data->V_OUT);
    }
}

void SPI2_IRQHandler(){
	//check if empty
	if(SPI2->SR & ((uint16_t)1 << 1) && comnet.len_tx > 0){
		SPI2->DR = *(comnet.buffer_tx++);
		comnet.len_tx -= 1;
	}
	if(SPI2->SR & ((uint16_t)1 << 0) && comnet.len_rx > 0){
			*(comnet.buffer_rx++) = (uint16_t)(SPI2->DR);
			comnet.len_rx -= 1;
		}
	if(comnet.len_rx <= 0 && comnet.len_tx <= 0){
		comnet.busy = 0;
		GPIOB->BSRR |= ((uint16_t)1 << comnet.CS_PIN); //pull CS 1 HIGH
		SPI2->CR2 &= ~(1 << 7);  //disa2le TEXEIE
		SPI2->CR2 &= ~(1 << 6);  //disable TEXEIE
	}

}

#endif /* SRC_SPI_H_ */

/*
 * configuration.h
 *
 *  Created on: Jun 7, 2025
 *      Author: domin
 */

#ifndef SRC_SPI_CONFIGURATION_H_
#define SRC_SPI_CONFIGURATION_H_

void gpio_cnfg(){
	RCC->AHB2ENR |= (1 << 1); // enable GPIOB CLK
	RCC->APB1ENR1 |= (1 << 14); // enable GPIOB CLK
}

void gpio_cnfg_direct(){
	 (*(volatile uint32_t*)0x4002104C) |= (1 << 1); // enable GPIOB CLK
	 (*(volatile uint32_t*)0x4002104C) |= (1 << 0); // enable GPIOA CLK
	 (*(volatile uint32_t*)0x40021058) |= (1 << 1); // enable GPIOB CLK
}

void spi_cnfg_direct(){
	uint32_t SET_state   = 0XA8000000;
	uint32_t reset_state = 0X3FFFFFF;
	(*(volatile uint32_t*)0x48000400) &= reset_state; //GPIOB->MODER RST
	(*(volatile uint32_t*)0x48000400) |= SET_state;  // GPIOB->MODERE SET AF

	uint32_t reset_AFHR = 0xFFFFF;
	uint32_t set_AFHR = 0x55500000;
	(*(volatile uint32_t*)0x48000424) &= reset_AFHR; //GPIOB->MODER RST
	(*(volatile uint32_t*)0x48000424) |= set_AFHR;  //set AFHR->AF5 for SPI2

	//configure CR1 and CR2
	uint16_t CR1_RESET = 0x3904;
	uint16_t CR1_PRESCALER =  0x30; // prescaler 256
	uint16_t CR1_MODE = 0x4;        // MASTER mode
	(*(volatile uint32_t*)0x40003800) &= CR1_RESET; //reset SPI2->CR1
	(*(volatile uint32_t*)0x40003800) |= CR1_PRESCALER | CR1_MODE; //SPI2->CR1 CNFG

	//configure CR2
	uint16_t CR2_RESET = 0x0700;
	uint16_t CR2_SET = 0x0700;
	(*(volatile uint32_t*)0x40003804) &= CR2_RESET;
	(*(volatile uint32_t*)0x40003804) |= CR2_SET;
}

void config_CS(){
	//CS pins PB 5 - 7, PB9, PB11, PB12
		uint32_t OM = 0x1;
		uint32_t RESET = 0x3;
		uint32_t PB5_SET = (OM << 10); //set to OM
		uint32_t PB6_SET = (OM << 12);
		uint32_t PB7_SET = (OM << 14);
		uint32_t PB9_SET = (OM << 18);
		uint32_t PB12_SET = (OM << 24);
		uint32_t PB11_SET = (OM << 22);
		uint32_t PB5_RESET = ~(RESET << 10);
		uint32_t PB6_RESET = ~(RESET << 12);
		uint32_t PB7_RESET = ~(RESET << 14);
		uint32_t PB9_RESET  = ~(RESET << 18);
		uint32_t PB12_RESET = ~(RESET << 24);
		uint32_t PB11_RESET = ~(RESET << 22);

		GPIOB->MODER &= (PB5_RESET & PB7_RESET & PB9_RESET & PB12_RESET & PB11_RESET);
		GPIOB->MODER |= (PB5_SET | PB7_SET | PB9_SET | PB12_SET | PB11_SET);

		//TOGGLE TO HIGH VIA BSRR



}

void spi_cnfg(){

	/*--------------------------------GPIOB MODE REGISTERS-------------------------------------------*/
	//GPIOB
	uint32_t PB15_SET_AF = ((uint32_t)1 << 31); //set to AF
	uint32_t PB14_SET_AF = ((uint32_t)1 << 29);
	uint32_t PB13_SET_AF = ((uint32_t)1 << 27);
	uint32_t PB1_SET_OUT = ((uint32_t)1  << 2);
	uint32_t PB15_RESET = ~(((uint32_t)1 << 31 | (uint32_t)1 << 30));
	uint32_t PB14_RESET = ~(((uint32_t)1 << 29 | (uint32_t)1 << 28));
	uint32_t PB13_RESET = ~(((uint32_t)1 << 27 | (uint32_t)1 << 26));
	uint32_t PB1_RESET  = ~(((uint32_t)1 << 3 | (uint32_t)1 << 2));

	GPIOB->MODER &= (PB15_RESET & PB14_RESET & PB13_RESET & PB1_RESET);
	GPIOB->MODER |= (PB15_SET_AF | PB14_SET_AF | PB13_SET_AF | PB1_SET_OUT);

	//AFR REG
	uint32_t PB15_AFHR_RESET = ~((uint32_t)1 << 31 | (uint32_t)1 << 30 | (uint32_t)1 << 29 | (uint32_t)1 << 28);
	uint32_t PB14_AFHR_RESET = ~((uint32_t)1 << 27 | (uint32_t)1 << 26 | (uint32_t)1 << 25 | (uint32_t)1 << 24);
	uint32_t PB13_AFHR_RESET = ~((uint32_t)1 << 23 | (uint32_t)1 << 22 | (uint32_t)1 << 21 | (uint32_t)1 << 20);
	uint32_t AF5 = ((uint32_t)1 << 0 | 1 << 2); // SET AF5

	GPIOB->AFR[1] &= (PB15_AFHR_RESET & PB14_AFHR_RESET & PB13_AFHR_RESET);
	GPIOB->AFR[1] |= (AF5 << 28 | AF5 << 24 | AF5 << 20);

	/*--------------------------------SPI CTRL REGISTERS-------------------------------------------*/

	//CR1 REG
	uint16_t CR1_RESET = ~(uint16_t)(
			(uint16_t)1 << 15 | (uint16_t)1 << 14 | (uint16_t)1 << 10|
			(uint16_t)1 << 9  | (uint16_t)1 << 7  | (uint16_t)1 << 6 |
			(uint16_t)1 << 5  | (uint16_t)1 << 4  | (uint16_t)1 << 3 |
			(uint16_t)1 << 1  | (uint16_t)1 << 0
	);

	uint32_t PCLK2 = 0x0;
	uint32_t PCLK4 = 0x1;
	uint32_t PCLK8 = 0x2;
	uint32_t PCLK16 = 0x3;
	uint32_t PCLK32 = 0x4;
	uint32_t PCLK64 = 0x5;
	uint32_t PCLK128 = 0x6;
	uint16_t CR1_PRESCALER = (PCLK32 << 3); // PRSCL 256
	uint16_t CR1_MODE = (1 << 2) | (0 << 1); // SPI2 MASTER mode

	SPI2->CR1 &= CR1_RESET;
	SPI2->CR1 |= CR1_PRESCALER | CR1_MODE;

	//CR2 REG
	uint16_t CR2_RESET = 0x0700;
	uint32_t DATA_SIZE = 0xB;//1011
	//uint16_t SPI2_DS = (0x7 << 8); // data size - 12 bits
	SPI2->CR2 &= CR2_RESET;
	SPI2->CR2 |= (0xB << 8) | (1u << 2);

	//enable interrupts
	NVIC_EnableIRQ(SPI2_IRQn);
	NVIC_SetPriority(SPI2_IRQn, 1);

}

void DMA_cnfg(){
	//DMAx_Channelx->CPAR = (uint32_t)&(SPI2->DR);
}

#endif /* SRC_SPI_CONFIGURATION_H_ */

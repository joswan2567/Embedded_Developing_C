/*
 * 004spi_tx_testing.c
 *
 *  Created on: Jan 24, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"
#include <string.h>

// 	  SPI2
/* PB15 - MOSI
 * PB14 - MISO
 * PB13 - SCK
 * PB12 - NSS */

void SPI2_GPIOInit(void);
void SPI2_Init(void);


int main(void){
	char data[] = "Qualé man";

	SPI2_GPIOInit(); // init GPIO pins SPI2 pins
	SPI2_Init();	 // init SPI2 peripheral parameters

	SPI_SSICfg(SPI2, ENABLE); // this makes NSS signal internally high and avoid MODF error

	SPI_PeripheralControl(SPI2, ENABLE);		   // enable the spi2 peripheral

	SPI_Send(SPI2, (uint8_t*)data, strlen(data));  // to send data

	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}

void SPI2_GPIOInit(void){
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_ALTFN;
	//SPI2Pins.GPIO_PinCfg.GPIO_PinAltFunMode = 0;
	SPI2Pins.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//SCK
	SPI2Pins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);

	//MOSI
	SPI2Pins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2Pins);

	/*//MISO
	SPI2Pins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);*/

}

void SPI2_Init(void){
	SPI_Handle_t  SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Cfg.SPI_BusCfg = SPI_BUS_CFG_FD;
	SPI2handle.SPI_Cfg.SPI_DeviceMode = SPI_DVC_MODE_MASTER;
	SPI2handle.SPI_Cfg.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPI_Cfg.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Cfg.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Cfg.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Cfg.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


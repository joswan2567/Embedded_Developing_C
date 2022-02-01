/*
 * 006spi_cmd_handling.c
 *
 *  Created on: Jan 26, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"
#include <string.h>

	// SPI1
/*
 * PA4 <-> NSS
 * PA5 <-> SCK
 * PA6 <-> MISO
 * PA7 <-> MOSI
 *
 */

// cmd codes
#define CMD_LED_CTRL			0x50
#define CMD_SENSOR_READ			0x51
#define CMD_LED_READ			0x52
#define CMD_PRINT				0x53
#define CMD_ID_READ				0x54

#define LED_ON					1
#define LED_OFF					0

// arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

#define BTN_PRESSED				LOW

void delay(uint32_t time);
void SPI1_GPIOInit(void);
void SPI1_Init(void);
void GPIOButton_Init(void);
uint8_t SPI_VerifyResponse(uint8_t ackByte);

int main(void){

	GPIOButton_Init();
	SPI1_GPIOInit();
	SPI1_Init();

	SPI_SSOECfg(SPI1, ENABLE);

	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	while(1)
		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			delay(500);

			SPI_PeripheralControl(SPI1, ENABLE);

			uint8_t cmdcode = CMD_LED_CTRL;
			uint8_t ackByte;
			SPI_Send(SPI1, &cmdcode, 1);

			SPI_Receive(SPI1, &dummy_read, 1); // rx dummy data for clear off RXNE

			SPI_Send(SPI1, &dummy_write, 1); //send dummy data

			SPI_Receive(SPI1, &ackByte, 1);

			if( SPI_VerifyResponse(ackByte) ){
				uint8_t args[2];
				args[0] = 9; //LED_PIN
				args[1] = LED_ON;

				SPI_Send(SPI1, args, 2);
			}

			while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

			SPI_PeripheralControl(SPI1, DISABLE);
		}
	return 0;
}

void delay(uint32_t time){
	uint32_t vlr = time * 500;
	for(uint32_t i = 0; i < vlr ; i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackByte){
	return ackByte == 0xF5 ? 1 : 0;
}
void SPI1_GPIOInit(void){
	GPIO_Handle_t SPI_gpio;

	SPI_gpio.pGPIOx = GPIOA;

	SPI_gpio.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_gpio.GPIO_PinCfg.GPIO_PinAltFunMode = 0;
	SPI_gpio.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_gpio.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_gpio.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	// NSS
	SPI_gpio.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPI_gpio);
	// SCK
	SPI_gpio.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPI_gpio);
	// MISO
	SPI_gpio.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPI_gpio);
	// MOSI
	SPI_gpio.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPI_gpio);

}

void SPI1_Init(void){
	SPI_Handle_t pHandle;

	pHandle.pSPIx = SPI1;

	pHandle.SPI_Cfg.SPI_DeviceMode = SPI_DVC_MODE_MASTER;
	pHandle.SPI_Cfg.SPI_DFF = SPI_DFF_8BITS;
	pHandle.SPI_Cfg.SPI_BusCfg = SPI_BUS_CFG_FD;
	pHandle.SPI_Cfg.SPI_SSM = SPI_SSM_DI;
	pHandle.SPI_Cfg.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	pHandle.SPI_Cfg.SPI_CPOL = SPI_CPOL_LOW;
	pHandle.SPI_Cfg.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&pHandle);
}

void GPIOButton_Init(void){
	GPIO_Handle_t pButton;

	pButton.pGPIOx = GPIOA;

	pButton.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
	pButton.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pButton.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(&pButton);
}


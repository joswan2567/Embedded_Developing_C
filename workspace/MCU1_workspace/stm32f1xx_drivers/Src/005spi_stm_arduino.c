/*
 * 005spi_stm_arduino.c
 *
 *  Created on: Jan 25, 2022
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

#define BTN_PRESSED			LOW

void delay(uint32_t time);
void SPI1_GPIOInit(void);
void SPI1_Init(SPI_Handle_t *pHandle);
void GPIOButton_Init(GPIO_Handle_t *pButton);

int main(void){
	GPIO_Handle_t GPIO_button;
	SPI_Handle_t SPI1_handle;

	GPIOButton_Init(&GPIO_button);
	SPI1_GPIOInit();
	SPI1_Init(&SPI1_handle);

	SPI_SSOECfg(SPI1, ENABLE);

	char txt[] = "What this is man";

	while(1)
		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			delay(500);

			SPI_PeripheralControl(SPI1, ENABLE);

			SPI_Send(SPI1, (uint8_t*)strlen(txt),1);
			SPI_Send(SPI1, (uint8_t*)txt, strlen(txt));

			while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

			SPI_PeripheralControl(SPI1, DISABLE);
		}
	return 0;
}

void delay(uint32_t time){
	uint32_t vlr = time * 500;
	for(uint32_t i = 0; i < vlr ; i++);
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

void SPI1_Init(SPI_Handle_t *pHandle){

	pHandle->pSPIx = SPI1;

	pHandle->SPI_Cfg.SPI_DeviceMode = SPI_DVC_MODE_MASTER;
	pHandle->SPI_Cfg.SPI_DFF = SPI_DFF_8BITS;
	pHandle->SPI_Cfg.SPI_BusCfg = SPI_BUS_CFG_FD;
	pHandle->SPI_Cfg.SPI_SSM = SPI_SSM_DI;
	pHandle->SPI_Cfg.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	pHandle->SPI_Cfg.SPI_CPOL = SPI_CPOL_LOW;
	pHandle->SPI_Cfg.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(pHandle);
}

void GPIOButton_Init(GPIO_Handle_t *pButton){

	pButton->pGPIOx = GPIOA;

	pButton->GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
	pButton->GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pButton->GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(pButton);
}

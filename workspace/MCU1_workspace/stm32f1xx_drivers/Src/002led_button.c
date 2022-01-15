/*
 * 002led_button.c
 *
 *  Created on: Jan 15, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"

#define HIGH           ENABLE
#define BTN_PRESSED    HIGH

void delay(uint32_t time){
	uint32_t vlr = time * 500;
	for(uint32_t i = 0; i < vlr ; i++);
}

int main(void){

	GPIO_Handle_t gpioLed, gpioBtn;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PIN_PD;


	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed);

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED){
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
			delay(500);
		}
	}
	return 0;
}


/*
 * 001led_toggle.c
 *
 *  Created on: Jan 14, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"

void delay(void){
	for(uint32_t i = 0; i < 500000 ; i++);
}

int main(void){

	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	//GPIO_Init(&gpioLed);

	gpioLed.pGPIOx->CR[1] &= ~(1 << 4*(gpioLed.GPIO_PinCfg.GPIO_PinNumber) );
	gpioLed.pGPIOx->CR[1] |= (0x03 << (4 * gpioLed.GPIO_PinCfg.GPIO_PinNumber));

	while(1){

		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}
	return 0;
}

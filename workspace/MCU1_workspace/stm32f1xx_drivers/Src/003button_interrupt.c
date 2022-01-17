/*
 * 003gpio_freq.c
 *
 *  Created on: Jan 17, 2022
 *      Author: José Wanderson
 */
#include <string.h>
#include "stm32f103xx.h"

#define HIGH           ENABLE
#define BTN_PRESSED    HIGH

void delay(uint32_t time){
	uint32_t vlr = time * 500;
	for(uint32_t i = 0; i < vlr ; i++);
}

int main(void){

	GPIO_Handle_t gpioLed, gpioBtn;
	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioBtn, 0, sizeof(gpioBtn));

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioBtn.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PIN_PUPD;


	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed);

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioBtn);

	//IRQ configuration
	GPIO_IRQPriorityCfg(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);
	GPIO_IRQITCfg(IRQ_NO_EXTI0, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_13);
	//delay(500);
}


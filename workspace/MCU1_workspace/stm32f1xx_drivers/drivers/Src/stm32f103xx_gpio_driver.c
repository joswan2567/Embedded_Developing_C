/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jan 10, 2022
 *      Author: José Wanderson
 */
#include "stm32f103xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if(EnOrDi){
		if      (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();

		return;
	}

	if      (pGPIOx == GPIOA) GPIOA_PCLK_DI();
	else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
	else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
	else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
	else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
	else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
	else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	/* 1 - cfg the cr of gpio pin

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQCfg(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}


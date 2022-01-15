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
	uint8_t aux1, aux2;
	aux1 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber / 8;
	aux2 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber % 8;

	if(pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_OUT){
		switch(pGPIOHandle->GPIO_PinCfg.GPIO_PinOPType){
			case GPIO_OP_TYPE_PP:
				if(pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_ALTFN){
					pGPIOHandle->pGPIOx->CR[aux1] &= ~(0x0F << (4 * aux2));
					pGPIOHandle->pGPIOx->CR[aux1] |= (0x08 << (4 * aux2));
				}
				else
					pGPIOHandle->pGPIOx->CR[aux1] &= ~(0x0F << (4 * aux2 ));
				break;
			case GPIO_OP_TYPE_OD:
				if(pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_ALTFN){
					pGPIOHandle->pGPIOx->CR[aux1] &= ~(0x0F << (4 * aux2));
					pGPIOHandle->pGPIOx->CR[aux1] |= (0x0C << (4 * aux2));
				}
				else{
					pGPIOHandle->pGPIOx->CR[aux1] &= ~(0x0F << (4 * aux2));
					pGPIOHandle->pGPIOx->CR[aux1] |= (0x04 << (4 * aux2));
				}
				break;
		}

		switch(pGPIOHandle->GPIO_PinCfg.GPIO_PinSpeed){
			case GPIO_SPEED_LOW:
				pGPIOHandle->pGPIOx->CR[aux1] |= (0x02 << (4 * aux2));
				break;
			case GPIO_SPEED_MEDIUM:
				pGPIOHandle->pGPIOx->CR[aux1] |= (0x01 << (4 * aux2));
				break;
			case GPIO_SPEED_FAST:
				pGPIOHandle->pGPIOx->CR[aux1] |= (0x03 << (4 * aux2));
				break;
		}
	}

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function reset peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if      (pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF) GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG) GPIOG_REG_RESET();
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function read data for the given GPIO pin
 *
 * @param[in]         - base address of the gpio port
 *
 * @param[in]         - base address of the gpio pin
 *
 * @return            - value reading in gpio pin
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function read data for the given GPIO port
 *
 * @param[in]         - base address of the gpio port
 *
 * @return            - values reading in gpio port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return (uint16_t)pGPIOx->IDR;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function write data for the given GPIO pin
 *
 * @param[in]         - base address of the gpio port
 *
 * @param[in]         - base address of the gpio pin
 *
 * @param[in]         - values for given in gpio pin
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function write data for the given GPIO port
 *
 * @param[in]         - base address of the gpio port
 *
 * @return         	  - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){
	pGPIOx->ODR = Value;
}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggle data for the given GPIO pin
 *
 * @param[in]         - base address of the gpio port
 *
 * @param[in]         - base address of the gpio pin
 *
 * @return         	  - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber );
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQCfg(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}


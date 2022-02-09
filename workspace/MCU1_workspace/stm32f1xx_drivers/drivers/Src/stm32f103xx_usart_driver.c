/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Feb 9, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx_usart_driver.h"

/*********************************************************************
 * @fn     		  	  - USART_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given USARTx
 *
 * @param[in]         - base address of the usart peripheral
 *
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi){
		if      (pUSARTx == USART1) USART1_PCLK_EN();
		else if (pUSARTx == USART2) USART2_PCLK_EN();
		else if (pUSARTx == USART3) USART3_PCLK_EN();
		else if (pUSARTx == UART4)  UART4_PCLK_EN();
		else if (pUSARTx == UART5)  UART5_PCLK_EN();

		return;
	}

	if      (pUSARTx == USART1) USART1_PCLK_DI();
	else if (pUSARTx == USART2) USART2_PCLK_DI();
	else if (pUSARTx == USART3) USART3_PCLK_DI();
	else if (pUSARTx == UART4)  UART4_PCLK_DI();
	else if (pUSARTx == UART5)  UART5_PCLK_DI();
}

/*********************************************************************
 * @fn      		  	 - USART_PeripheralControl
 *
 * @brief             - This function enORdi peripheral clock for the given USARTx
 *
 * @param[in]         - base address of the USARTx
 *
 * @param[in]         - enORdi
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi)
		pUSARTx->SR |= (1 << USART_CR1_UE);
	else
		pUSARTx->SR &= ~(1 << USART_CR1_UE);
}

/*********************************************************************
 * @fn      		  	 - USART_GetFlagStatus
 *
 * @brief             - This function return status of flag
 *
 * @param[in]         - structure register of usartx
 *
 * @param[in]         - macros flag name
 *
 * @return            - status of flag
 *
 * @Note              - none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){
	return (pUSARTx->SR & StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - This function clear status of flag
 *
 * @param[in]         - structure register of usartx
 *
 * @param[in]         - macros flag name
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){
	pUSARTx->SR &= ~(1 << StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptCfg
 *
 * @brief             - This function clear status of flag
 *
 * @param[in]         - structure register of usartx
 *
 * @param[in]         - macros flag name
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_IRQInterruptCfg(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	}
	else {
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityCfg
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void  USART_IRQPriorityCfg(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

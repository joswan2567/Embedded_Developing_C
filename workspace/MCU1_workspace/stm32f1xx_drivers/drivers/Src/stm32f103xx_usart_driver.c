/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Feb 9, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx_usart_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){ }

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){ }

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){ }
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){ }

uint8_t USART_IRQInterruptCfg(uint8_t IRQNumber, uint8_t EnOrDi){ }
uint8_t USART_IRQPriorityCfg(uint8_t IRQNumber, uint32_t IRQPriority){ }

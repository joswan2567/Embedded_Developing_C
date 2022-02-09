/*
 * stm32f103xx_usart_driver.h
 *
 *  Created on: Feb 9, 2022
 *      Author: José Wanderson
 */
#include "stm32f103xx.h"

#ifndef INC_STM32F103XX_USART_DRIVER_H_
#define INC_STM32F103XX_USART_DRIVER_H_

/*
 * This is a Configuration Structure for a USARTx peripheral
 */
typedef struct{
	uint8_t  USART_Mode;               /*!<possible modes for device> */
	uint32_t USART_Baud;               /*!<possible baud rate for communication>*/
	uint8_t  USART_NoOfStopBits;	   /*!<TODO>*/
	uint8_t  USART_WordLength;		   /*!<possible size data 8 or 9 bits >*/
	uint8_t  USART_ParityControl;	   /*!<TODO> */
	uint8_t  USART_HWFlowControl;	   /*!<TODO> */

}USART_Cfg_t;

/*
 * This is a Handle structure for a USARTx peripheral
 */
typedef struct{
	USART_RegDef_t *pUSARTx;              /*!< This holds the base addr of the USARTx port to which the pin belongs */
	USART_Cfg_t     USART_Cfg;			  /*!< This holds USARTx configuration settings */

}USART_Handle_t;


/*****************************************************************
 *				 APIs supported by this driver                   *
 *  For more info. about the APIs check the function definition  *
 *****************************************************************/

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

void USART_IRQInterruptCfg(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityCfg(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F103XX_USART_DRIVER_H_ */

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

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0


#define 	USART_EVENT_TX_CMPLT   0
#define		USART_EVENT_RX_CMPLT   1
#define		USART_EVENT_IDLE      2
#define		USART_EVENT_CTS       3
#define		USART_EVENT_PE        4
#define		USART_ERR_FE     	5
#define		USART_ERR_NE    	 6
#define		USART_ERR_ORE    	7
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

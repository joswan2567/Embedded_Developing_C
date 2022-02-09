/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Feb 9, 2022
 *      Author: José Wanderson
 */


/*
 * This is a Configuration Structure for a USARTx peripheral
 */
typedef struct{
	uint8_t  USART_Mode;               /*!<possible modes for device> */
	uint32_t USART_Baud;               /*!<possible baud rate for communication>*/
	uint8_t  USART_NoOfStopBits;	   /*!<possible speed clock>*/
	uint8_t  USART_WordLength;		   /*!<possible size data 8 or 9 bits >*/
	uint8_t  USART_ParityControl;	   /*!<TODO> */
	uint8_t  USART_HWFlowControl;	   /*!<TODO> */

}USART_Cfg_t;

/*
 * This is a Handle structure for a USARTx peripheral
 */
typedef struct{
	USART_RegDef_t *pUSARTx;              /*!< This holds the base addr of the SPIx port to which the pin belongs */
	USART_Cfg_t     USART_Cfg;			  /*!< This holds SPIx configuration settings */

}USART_Handle_t;


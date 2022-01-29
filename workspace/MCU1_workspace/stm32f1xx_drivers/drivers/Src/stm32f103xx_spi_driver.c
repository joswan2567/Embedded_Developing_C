/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jan 22, 2022
 *      Author: José Wanderson
 */
#include "stm32f103xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIx
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi){
		if      (pSPIx == SPI1) SPI1_PCLK_EN();
		else if (pSPIx == SPI2) SPI2_PCLK_EN();
		else if (pSPIx == SPI3) SPI3_PCLK_EN();

		return;
	}

	if      (pSPIx == SPI1) SPI1_PCLK_DI();
	else if (pSPIx == SPI2) SPI2_PCLK_DI();
	else if (pSPIx == SPI3) SPI3_PCLK_DI();
}

/*********************************************************************
* @fn      		  	 - SPI_Init
*
* @brief             - This function init peripheral clock for the given SPIx
*
* @param[in]         - handle SPIx
*
* @return            - none
*
* @Note              - none
*/
void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE); // enable peripheral clock

	// cfg the SPI_CR1 register
	uint32_t tempreg = 0;

	// cfg the device mode
	tempreg |= pSPIHandle->SPI_Cfg.SPI_DeviceMode << SPI_CR1_MSTR;

	// cfg the bus cfg
	if(pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_FD){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE); // bidi mode clear
	}
	else if (pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_HD)
		tempreg |=  (1 << SPI_CR1_BIDIMODE); // bidi mode be set
	else if (pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_SPLEX_RX_ONLY){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE); // bidi mode clear
		tempreg |=  (1 << SPI_CR1_RXONLY); // rxonly bit be set
	}

	// cfg speedclk
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_SclkSpeed << SPI_CR1_BR);

	// cfg dff
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_DFF << SPI_CR1_DFF);

	// cfg cpol
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_CPOL << SPI_CR1_CPOL);

	// cfg cpha
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_CPHA << SPI_CR1_CPHA);

	// cfg ssm
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 |= tempreg;

}
 /*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function reset peripheral clock for the given SPIx
 *
 * @param[in]         - base address of the SPIx
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if      (pSPIx == SPI1) SPI1_REG_RESET();
	else if (pSPIx == SPI2) SPI2_REG_RESET();
	else if (pSPIx == SPI3) SPI3_REG_RESET();
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	return (pSPIx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}
/*********************************************************************
* @fn      		  	 - SPI_Send
*
* @brief             - This function reset peripheral clock for the given SPIx
*
* @param[in]         - base address of the SPIx
*
* @param[in]         - base address of data
*
* @param[in]         - size of data
*
* @return            - none
*
* @Note              - This is blocking  call
*/
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Size){
	while(Size){
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); //wait until TXE is set

		// check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF))){ 		// 16 bit DFF
			pSPIx->DR = *((uint16_t*) pTxBuffer); 		// load the data in to the DR
			Size -= 2;
			(uint16_t*)pTxBuffer++;
		}else{											// 8 bit DFF
			pSPIx->DR = *(pTxBuffer); 					// load the data in to the DR
			Size--;
			pTxBuffer++;
		}
	}

}

/*********************************************************************
* @fn      		  	 - SPI_Receive
*
* @brief             - This function reset peripheral clock for the given SPIx
*
* @param[in]         - base address of the SPIx
*
* @param[in]         - base address of data
*
* @param[in]         - size of data
*
* @return            - none
*
* @Note              - This is blocking  call
*/
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Size){
	while(Size){
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET); //wait until RXE is set

		// check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF))){ 		// 16 bit DFF
			*((uint16_t*) pRxBuffer) = pSPIx->DR; 		// load the data from DR to Rxbuffer addr
			Size -= 2;
			(uint16_t*)pRxBuffer++;
		}else{											// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR; 					// load the data from DR to Rxbuffer addr
			Size--;
			pRxBuffer++;
		}
	}

}
/*********************************************************************
* @fn      		  	 - SPI_PeripheralControl
*
* @brief             - This function enORdi peripheral clock for the given SPIx
*
* @param[in]         - base address of the SPIx
*
* @param[in]         - base address of data
*
* @param[in]         - size of data
*
* @return            - none
*
* @Note              - none
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/*********************************************************************
* @fn      		  	 - SPI_SSICfg
*
* @brief             - This function enORdi register SSI for SPIx
*
* @param[in]         - base address of the SPIx
*
* @param[in]         - enORdi
*
* @return            - none
*
* @Note              - none
*/
void SPI_SSICfg(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

/*********************************************************************
* @fn      		  	 - SPI_SSOECfg
*
* @brief             - This function enORdi register SSOE for SPIx
*
* @param[in]         - base address of the SPIx
*
* @param[in]         - enORdi
*
* @return            - none
*
* @Note              - none
*/
void SPI_SSOECfg(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

/*********************************************************************
* @fn      		  	 - SPI_IRQITCfg
*
* @brief             - This function enORdi interruption for SPIx
*
* @param[in]         - number of IRQ
*
* @param[in]         - enORdi
*
* @return            - none
*
* @Note              - none
*/
void SPI_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi){
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
* @fn      		  	 - SPI_IRQPriorityCfg
*
* @brief             - This function set priority for IRQ SPIx
*
* @param[in]         - number of IRQ
*
* @param[in]         - IRQ priority
*
* @return            - none
*
* @Note              - none
*/
void SPI_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/*********************************************************************
* @fn      		  	 - SPI_IRQHandling
*
* @brief             - This function set priority for IRQ SPIx
*
* @param[in]         - number of IRQ
*
* @param[in]         - IRQ priority
*
* @return            - none
*
* @Note              - none
*/
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}

/*********************************************************************
* @fn      		  	 - SPI_SendIT
*
* @brief             - This function send data with it
*
* @param[in]         - handle SPIx
*
* @param[in]         - pointer data
*
* @param[in]         - size data
*
* @return            - none
*
* @Note              - none
*/
uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Size){

	if(!pSPIHandle->TXState){
		// 1. Save the TX buf addr and size info.
		pSPIHandle->pTXBuf = pTxBuffer;
		pSPIHandle->TXLen = Size;

		// 2. Mark the SPI state as busy
		pSPIHandle->TXState = SPI_BSY_TX;

		// 3. Enable the TXEIE ctrl bit to get it whenever TXE flag is set is ISR
		pSPIHandle->pSPIx->CR2 = (1 << SPI_CR2_TXEIE);

		// 4. Data transmission will be handled by the ISR code (will implement later)
	}

	return pSPIHandle->TXState;
}

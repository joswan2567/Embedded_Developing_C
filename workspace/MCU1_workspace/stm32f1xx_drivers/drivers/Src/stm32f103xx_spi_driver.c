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

	// cfg the SPI_CR1 register
	uint32_t tempreg = 0;

	// cfg the device mode
	tempreg |= pSPIHandle->SPI_Cfg.SPI_DeviceMode << SPI_CR1_MSTR;

	// cfg the bus cfg
	if(pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_FD){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE); // bidi mode clear
		pSPIHandle->pSPIx->CR1 |= tempreg;
	}
	else if (pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_HD)
		tempreg |=  (1 << SPI_CR1_BIDIMODE); // bidi mode be set
	else if (pSPIHandle->SPI_Cfg.SPI_BusCfg == SPI_BUS_CFG_SPLEX_RX_ONLY){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE); // bidi mode clear
		tempreg |=  (1 << SPI_CR1_RXONLY); // rxonly bit be set
	}

	// cfg speedclk
	pSPIHandle->pSPIx->CR1 &= ~(0x07 << SPI_CR1_BR);
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_SclkSpeed << SPI_CR1_BR);

	// cfg dff
	pSPIHandle->pSPIx->CR1 &= ~(0x03 << SPI_CR1_DFF);
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_DFF << SPI_CR1_DFF);

	// cfg cpol
	pSPIHandle->pSPIx->CR1 &= ~(0x01 << SPI_CR1_CPOL);
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_CPOL << SPI_CR1_CPOL);

	// cfg cpha
	pSPIHandle->pSPIx->CR1 &= ~(0x01 << SPI_CR1_CPHA);
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_CPHA);

	// cfg ssm
	pSPIHandle->pSPIx->CR1 &= ~(0x01 << SPI_CR1_SSM);
	tempreg |= (pSPIHandle->SPI_Cfg.SPI_SSM);

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
	uint8_t TX = 1;
	while(Size){
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); //wait until TXE is set

		// check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF))){ 		// 16 bit DFF
			pSPIx->DR = *((uint16_t*) pTxBuffer); 		// load the data in to the DR
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}else{											// 8 bit DFF
			pSPIx->DR = *(pTxBuffer); 					// load the data in to the DR
			Len--;
			pTxBuffer++;
		}
	}

}
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Size){

}
/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi){

}
void SPI_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}

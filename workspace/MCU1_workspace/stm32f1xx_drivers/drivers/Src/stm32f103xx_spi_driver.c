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

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

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

/*
 * Data send and receive
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Size){

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

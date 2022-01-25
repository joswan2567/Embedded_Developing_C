/*
 * stm32f1xx_spi_driver.h
 *
 *  Created on: Jan 22, 2022
 *      Author: José Wanderson
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"

/*
 * This is a Configuration Structure for a SPIx peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;           /*!<possible modes for device> */
	uint8_t SPI_BusCfg;               /*!<possible types for communication>*/
	uint8_t SPI_SclkSpeed;			  /*!<possible speed clock>*/
	uint8_t SPI_DFF;				  /*!<possible size data 8 or 16 bits >*/
	uint8_t SPI_CPOL;				  /*!<TODO> */
	uint8_t SPI_CPHA;				  /*!<TODO> */
	uint8_t SPI_SSM;				  /*!<select slave for software or hardware >*/

}SPI_Cfg_t;

/*
 * This is a Handle structure for a SPIx peripheral
 */
typedef struct{
	SPI_RegDef_t *pSPIx;              /*!< This holds the base addr of the SPIx port to which the pin belongs */
	SPI_Cfg_t     SPI_Cfg;			  /*!< This holds SPIx configuration settings */

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DVC_MODE_MASTER						1
#define SPI_DVC_MODE_SLAVE						0

/*
 * @SPI_BusCfg
 */
#define SPI_BUS_CFG_FD							1
#define SPI_BUS_CFG_HD							2
#define SPI_BUS_CFG_SPLEX_RX_ONLY				3

/*
 * @SPI_SlckSpeed
 */
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH							HIGH
#define SPI_CPOL_LOW							LOW

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH							HIGH
#define SPI_CPHA_LOW							LOW

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN								1
#define SPI_SSM_DI								0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG							(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG							(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG							(1 << SPI_SR_BSY)


/*****************************************************************
 *				 APIs supported by this driver                   *
 *  For more info. about the APIs check the function definition  *
 *****************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Size);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Size);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control API's
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSICfg(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */

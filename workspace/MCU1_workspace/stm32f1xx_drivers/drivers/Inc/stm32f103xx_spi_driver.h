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

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */

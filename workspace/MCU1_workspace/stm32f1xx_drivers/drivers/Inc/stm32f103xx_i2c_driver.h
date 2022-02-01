/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: Jan 31, 2022
 *      Author: José Wanderson
 */

#ifndef INC_STM32F103XX_I2C_DRIVER_H_
#define INC_STM32F103XX_I2C_DRIVER_H_
#include "stm32f103xx.h"

/*
 * This is a Configuration Structure for a I2Cx peripheral
 */
typedef struct{
	uint8_t I2C_SCLSpeed;             /*!<possible speed for SCL > */
	uint8_t SPI_DeviceAddr;           /*!<(if device mode slave) device addr >*/
	uint8_t SPI_ACKControl;			  /*!<Control ACK>*/
	uint8_t SPI_FMDutyCycle;		  /*!<TODO: >*/

}I2C_Cfg_t;

/*
 * This is a Handle Structure for a I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Cfg_t	 I2C_Cfg;

}I2C_Handle_t;

#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */

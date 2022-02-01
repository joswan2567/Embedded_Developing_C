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
	uint32_t I2C_SCLSpeed;             /*!<possible speed for SCL > */
	uint8_t I2C_DeviceAddr;           /*!<(if device mode slave) device addr >*/
	uint8_t I2C_ACKControl;			  /*!<Control ACK>*/
	uint8_t I2C_FMDutyCycle;		  /*!<TODO: >*/

}I2C_Cfg_t;

/*
 * This is a Handle Structure for a I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Cfg_t	 I2C_Cfg;

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */

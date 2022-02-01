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

/*****************************************************************
 *				 APIs supported by this driver                   *
 *  For more info. about the APIs check the function definition  *
 *****************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */


/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Other Peripheral Control API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */

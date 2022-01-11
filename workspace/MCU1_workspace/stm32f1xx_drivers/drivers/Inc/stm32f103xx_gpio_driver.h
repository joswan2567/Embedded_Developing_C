/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jan 10, 2022
 *      Author: José Wanderson
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"
/*
 * This is a Configuration Structure for a GPIO pin
 */

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinCfg_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;              /*!< This holds the base addr of the GPIO port to which the pin belongs */
	GPIO_PinCfg_t GPIO_PinCfg;          /*!< This holds GPIO pin configuration settings */

}GPIO_Handle_t;

#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */

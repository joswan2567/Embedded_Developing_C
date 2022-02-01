/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Jan 31, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx_i2c_driver.h"

// This is a weak implementation, the application may override this function.
__weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2Cx
 *
 * @param[in]         - base address of the i2c peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi){
		if      (pI2Cx == I2C1) I2C1_PCLK_EN();
		else if (pI2Cx == I2C2) I2C2_PCLK_EN();

		return;
	}

	if      (pI2Cx == I2C1) I2C1_PCLK_DI();
	else if (pI2Cx == I2C2) I2C2_PCLK_DI();
}

/*********************************************************************
* @fn      		  	 - I2C_Init
*
* @brief             - This function init peripheral clock for the given I2Cx
*
* @param[in]         - handle I2Cx
*
* @return            - none
*
* @Note              - none
*/
void I2C_Init(I2C_Handle_t *pI2CHandle){

}

/*********************************************************************
* @fn	      		 - I2C_DeInit
*
* @brief             - This function reset peripheral clock for the given I2Cx
*
* @param[in]         - base address of the I2Cx
*
* @return            - none
*
* @Note              - none
*/
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if      (pI2Cx == I2C1) I2C1_REG_RESET();
	else if (pI2Cx == I2C2) I2C2_REG_RESET();
}

/*
 * Data send and receive
 */


/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi){}
void I2C_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority){}

/*********************************************************************
* @fn      		  	 - I2C_PeripheralControl
*
* @brief             - This function enables or disables peripheral clock for the given I2Cx
*
* @param[in]         - base address of the I2Cx
*
* @param[in]         - enORdi
*
* @return            - none
*
* @Note              - none
*/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi)
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
}

/*
 * Application Callback
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){}

/*
 * 007i2c_master_tx_testing.c
 *
 *  Created on: Feb 4, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"
#include <string.h>

/*
 * PB6 < - > SCL
 * PB7 < - > SDA
 */
#define SLAVE_ADDR 0x61

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(uint32_t time);
void I2C1_GPIOInits(void);
void I2C1_Init(void);
void GPIOButton_Init(void);

int main(void){
	I2C1_GPIOInits(); 						// i2c pin inits
	I2C1_Init(); 							// i2c peripheral cfg
	I2C_PeripheralControl(I2C1, ENABLE);	// enable the i2c peripheral

	while(1)
		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
		delay(500);

		// send some data to slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*) some_data), SLAVE_ADDR);
	}

	return 0;
}

void delay(uint32_t time){
	uint32_t vlr = time * 500;
	for(uint32_t i = 0; i < vlr ; i++);
}

void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinCfg.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PIN_PUPD;
	I2CPins.GPIO_PinCfg.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Init(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Cfg.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_Cfg.I2C_DeviceAddr = 0x61; //this is not necessary, code is master
	I2C1Handle.I2C_Cfg.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Cfg.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIOButton_Init(void){
	GPIO_Handle_t pButton;

	pButton.pGPIOx = GPIOA;

	pButton.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
	pButton.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pButton.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(&pButton);
}

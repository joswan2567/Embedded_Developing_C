/*
 * 007i2c_master_tx_testing.c
 *
 *  Created on: Feb 4, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx.h"
#include <string.h>
extern void initialise_monitor_handles();

uint8_t rxComplt = RESET;

/*
 * PB6 < - > SCL
 * PB7 < - > SDA
 */
#define SLAVE_ADDR 0x61

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";
uint8_t len, cmd_code, rcv_buf[32];

void delay(uint32_t time);
void I2C1_GPIOInits(void);
void I2C1_Init(void);
void GPIOButton_Init(void);

int main(void){
	I2C1_GPIOInits(); 						// i2c pin inits
	I2C1_Init(); 							// i2c peripheral cfg

	//I2C IRQ cfg
	I2C_IRQITCfg(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITCfg(IRQ_NO_I2C1_ER, ENABLE);

	GPIOButton_Init();						// init button interrupt
	I2C_PeripheralControl(I2C1, ENABLE);	// enable the i2c peripheral

	while(1)
		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
		delay(500);

		// send some data to slave
		cmd_code = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle, &cmd_code, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReadDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		cmd_code = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &cmd_code, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReadDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		while(rxComplt != SET);

		rcv_buf[len + 1] = '\0';

		printf("Data : %s", rcv_buf);

		rxComplt = RESET;
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

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	if(AppEv == I2C_EV_TX_CMPLT)
	 {
		 printf("Tx is completed\n");
	 }else if (AppEv == I2C_EV_RX_CMPLT)
	 {
		 printf("Rx is completed\n");
		 rxComplt = SET;
	 }else if (AppEv == I2C_ERROR_AF)
	 {
		 printf("Error : Ack failure\n");
		 //in master ack failure happens when slave fails to send ack for the byte
		 //sent from the master.
		 I2C_CloseTX(pI2CHandle);

		 //generate the stop condition to release the bus
		 I2C_GenerateStopCondition(I2C1);

		 //Hang in infinite loop
		 while(1);
	 }
}

/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Jan 31, 2022
 *      Author: José Wanderson
 */

#include "stm32f103xx_i2c_driver.h"

#define I2C_PHASE_READ			1
#define I2C_PHASE_WRITE			0

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t R_W);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

// This is a weak implementation, the application may override this function.
__weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){}

uint16_t AHB_PreScaler[9]  = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};


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

uint32_t RCC_GetPLLOutputClock(){
	return 0;
}

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SysClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0){
		SysClk = 16000000;
	}
	else if(clksrc == 1){
		SysClk = 8000000;
	}
	else if(clksrc == 2){
		SysClk = RCC_GetPLLOutputClock();
	}

	// for AHB PreScaler
	temp = ((RCC->CFGR >> 4) & 0x0F);

	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// for APB1 PreScaler
	temp = ((RCC->CFGR >> 10) & 0x07);

	if(temp < 4){
		apb1p = 1;
	}
	else{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SysClk * apb1p) / ahbp; // (SysClk / ahbp) / apb1p;

	return pclk1;
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
	uint32_t tempreg = 0;

	//enable the clock for the I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ack control
	tempreg |= pI2CHandle->I2C_Cfg.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// cfg the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x03F); // isso deixa somente o 5 bits primeiros bit visiveis

	//program the device own addr
	tempreg |= pI2CHandle->I2C_Cfg.I2C_DeviceAddr << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Cfg.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Cfg.I2C_SCLSpeed);
		tempreg |=(ccr_value & 0xFFF); // aqui deixamos visiveis somente os 12 primeiros bits
	}
	else{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Cfg.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Cfg.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Cfg.I2C_SCLSpeed);
		}
		else{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Cfg.I2C_SCLSpeed);
		}
		tempreg |=(ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Cfg
	if(pI2CHandle->I2C_Cfg.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
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

/*********************************************************************
 * @fn	      		 - I2C_MasterSendData
 *
 * @brief             - This function send data for I2Cx
 *
 * @param[in]         - handle of the I2Cx
 *
 * @param[in]         - base address of data
 *
 * @param[in]         - size of data
 *
 * @param[in]         - address of slave device
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Size, uint8_t SlaveAddr, uint8_t Sr){
	// Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)); // EV5

	// Send the addr of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_PHASE_WRITE);

	// Confirm that addr phase is done by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // EV6

	// clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	// send the data until Size becomes 0
	while(Size){
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) ; //wait until TXE is set // EV8										// 8 bit DFF
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Size--;
	}

	// when size becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // EV8_2

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); // EV8_2

	// Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t R_W){
	SlaveAddr = SlaveAddr << 1; 	// open for a bit r/w;
	if     (!R_W) SlaveAddr &= ~(0x01); 	// SlaveAddr is SlaveAddr + w bit
	else if( R_W) SlaveAddr |=  (0x01); 	// SlaveAddr is SlaveAddr + r bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;

	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		// device in master mode
		if(pI2CHandle->TXRXState == I2C_BSY_IN_RX){
			if(pI2CHandle->RXSize == 1){
				// first disable the ack
				I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);

				// clear the ADDR flag(read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else{
			// clear the ADDR flag(read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else{
		// device in slave mode

		// clear the ADDR flag(read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

/*********************************************************************
 * @fn	      		 - I2C_MasterReadData
 *
 * @brief             - This function read data for I2Cx
 *
 * @param[in]         - handle of the I2Cx
 *
 * @param[in]         - base address of data
 *
 * @param[in]         - size of data
 *
 * @param[in]         - address of slave device
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_MasterReadData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Size, uint8_t SlaveAddr){
	// Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)); // EV5

	// Send the addr of the slave with r/w bit set to r(1) (total 8 bits)
	I2C_ExecAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_PHASE_READ);

	//wait until address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // EV6

	// procedure to read only 1 byte from slave
	if (Size == 1){
		// Disable ACK
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DI);

		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until rxne becomes
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	// procedure to read data from slave when Size > 1
	if(Size > 1){
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read the data until Size becomes zero
		for(uint32_t i = Size ; i > 0; i--){
			// wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if( i == 2){	// if last 2 bytes are remaining
				// clear the ACK bit
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DI);

				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			// read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	// re-enable ACK
	if(pI2CHandle->I2C_Cfg.I2C_ACKControl == I2C_ACK_EN)
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_EN);
}

static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi) pI2Cx->CR1 |=  (I2C_ACK_EN << I2C_CR1_ACK);
	else       pI2Cx->CR1 &= ~(I2C_ACK_EN << I2C_CR1_ACK);
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file [/] check

 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Size, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TXRXState;

	if( (busystate != I2C_BSY_IN_TX) && (busystate != I2C_BSY_IN_RX))
	{
		pI2CHandle->pTXBuf = pTxBuffer;
		pI2CHandle->TXLen = Size;
		pI2CHandle->TXRXState = I2C_BSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;

}

uint8_t I2C_MasterReadDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Size, uint8_t SlaveAddr){

	return 0;
}

/*********************************************************************
 * @fn      		  - I2C_IRQITCfg
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQITCfg(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityCfg
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQPriorityCfg(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	// Interrupt handling for both master and slave mode of device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3){
		// The interrupt is generated because of SB event
		// this block will not be executed in slave mode because for slave SB is always zero
		if(pI2CHandle->TXRXState == I2C_BSY_IN_TX)
			I2C_ExecAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->I2C_Cfg.I2C_DeviceAddr, I2C_PHASE_WRITE);
		else if(pI2CHandle->TXRXState == I2C_BSY_IN_RX)
			I2C_ExecAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->I2C_Cfg.I2C_DeviceAddr, I2C_PHASE_READ);
	}

	// Handle for interrupt generated by ADDR event
	// Note: When master mode: Addr is sent
	// 		 When slave mode : Addr matched with own addr
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3){
		// Interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// Handle for interrupt generated by BTF(Byte transfer finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3){
		// BTF flag is set
		if(pI2CHandle->TXRXState == I2C_BSY_IN_TX){
			// make sure that TXE is also set.
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				// BTF, TXE = 1
				if(!pI2CHandle->TXLen){
					// generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// reset all the member elements of the handle structure
					I2C_CloseTX(pI2CHandle);

					// notify the application about transmission complete
					I2C_AppEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TXRXState == I2C_BSY_IN_RX){ /**/ }
	}

	// Handle for interrupt generated by STOPF event
	// Note: Stop detection flag is applicable only slave mode. For master this flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF); // 1) read SR1

	if(temp1 && temp3){
		// STOPF flag is set
		// clear the STOPF ( i.e 1) read SR1 2) write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify the application about stop is detected
		I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3){
		// check foe device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// TXE flag is set
			// we have to the data transmission
			if(pI2CHandle->TXRXState == I2C_BSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
	}

	// Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3){
		//check device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// the device is master

			// RXNE flag is set
			if(pI2CHandle->TXRXState == I2C_BSY_IN_RX){

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TXLen > 0){
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTXBuf);
		pI2CHandle->TXLen -- ;
		pI2CHandle->pTXBuf++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	// we have to do the data reception
	if(pI2CHandle->RXSize == 1){
		*pI2CHandle->pRXBuf = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RXLen--;
	}

	if(pI2CHandle->RXSize > 1){
		if(pI2CHandle->RXLen == 2){
			// clear the ack bit
			I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
		}
		// read DR
		*pI2CHandle->pRXBuf = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRXBuf++;
		pI2CHandle->RXLen--;
	}

	if(pI2CHandle->RXSize == 0){
		// close the I2C RX and notify app

		// generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// close the I2C rx
		I2C_CloseRX(pI2CHandle);

		// notify app
		I2C_AppEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

	}
}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - none

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);


		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

	/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);

	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

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
	return (pI2Cx->SR1 & FlagName); // test : option = 	return (pI2Cx->SR1 & FlagName) ? FLAG_SET : FLAG_RESET;
}

void I2C_CloseTX(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TXRXState = I2C_READY;
	pI2CHandle->pTXBuf = NULL;
	pI2CHandle->TXLen = 0;
}

void I2C_CloseRX(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	// Implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->TXRXState = I2C_READY;
	pI2CHandle->pRXBuf = NULL;
	pI2CHandle->RXLen = 0;
	pI2CHandle->RXSize = 0;
	if(pI2CHandle->I2C_Cfg.I2C_ACKControl == I2C_ACK_EN)
		I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
}


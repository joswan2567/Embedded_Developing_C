#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stdio.h>
#include <string.h>

int pchar(int c){
    //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    while(!(USART1->SR & USART_FLAG_TXE));
    USART1->DR = (c & 0xff);
    //USART1->DR = 'A';
    //USART_SendData(USART1, (uint8_t)c);
    return 0;
}

int getchar(void){
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return USART1->DR & 0xFF;
}
// Timer Code (5)
static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay != 0);
}

void SysTick_Handler(void)
{
    if (TimingDelay != 0x00)
        TimingDelay--;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1);
}
#endif

int main(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    // Init USART1_Tx
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Init USART1_Rx
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, & GPIO_InitStruct);

    // Init USART1
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;/*
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);

    const char t[11] = "Hello World";

    while (1){
        int i = 0;
        while (i < 5){
	    pchar(t[i++]);
	    //pchar((char)'s');
	    //pchar(i++);
	    Delay(50);
        }
	//pchar(0x0a);
        Delay(250);
   }
}

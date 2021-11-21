#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure, GPIO_InitStructure1;

    // Button
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // LED
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure1);
    GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure1);
    //if(SysTick_Config(SystemClockCore / 1000));
    //    while (1);
    while (1) {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
	    {
    	    GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
        }
	    else
	    {
	        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
	    }
    }
   return 0;
}
static __IO uint32_t TimingDelay;

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

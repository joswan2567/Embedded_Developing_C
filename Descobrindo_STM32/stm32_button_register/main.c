#include <stm32f10x.h>

void Delay(uint32_t nTime);

int main(void)
{
    SystemInit();
    // Clock GPIOA,B
    RCC->APB2ENR|=((1<<2) | (1<<3));

    // LED
    GPIOB->CRH&=~((1<<30) | (1<<31)); //output mode
    GPIOB->CRH|=(1<<28) | (1<<27);

    // BUTTON
    GPIOA->CRL&= ~(0xf<<4);
    GPIOA->CRL|=(8<<1); // input mode
    GPIOA->ODR|=(1<<1);
    if(SysTick_Config(SystemCoreClock / 1000))
	while (1);
    while (1) {
        if((GPIOA->IDR & (1<<0)))
	    //GPIOB->BSRR |= (1<<15);
	    0x40011018 |= (1<<15);
	else
    	    GPIOB->BSRR |= (1<<15) <<16;
    }
    return 0;
}
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

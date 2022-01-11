/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jan 10, 2022
 *      Author: José Wanderson
 */

#include <stdint.h>
#include "stm32f103xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    /* Loop forever */
	for(;;);
}

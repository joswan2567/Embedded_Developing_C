/*
 * stm32f103xx.h
 *
 *  Created on: Jan 9, 2022
 *      Author: José Wanderson
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
/*
 *  base addr of FLASH and SRAM  memories
 */

#define FLASH_BASE_ADDR      0x08000000U     /*base address flash bluepill*/
#define SRAM_BASE_ADDR       0x20000000U     /*base address sram bluepill*/
#define ROM_BASE_ADDR        0x1FFFF000U     /*base address system memory bluepill*/
#define SRAM                 SRAM_BASE_ADDR

#endif /* INC_STM32F103XX_H_ */

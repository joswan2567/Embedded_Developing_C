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
#define SRAM1_BASE_ADDR       0x20000000U     /*base address sram bluepill*/
//#define SRAM2_BASE_ADDR       0x20000000U     /*base address sram bluepill*/
#define ROM_BASE_ADDR        0x1FFFF000U     /*base address system memory bluepill*/
#define SRAM                 SRAM1_BASE_ADDR

/*
 * AHBx and APBX Bus Peripheral base addr
 */

#define PERIPH_BASE          0x40000000U
#define APB1_PERIPH_BASE     PERIPH_BASE
#define APB2_PERIPH_BASE     0x40010000U
#define AHB1_PERIPH_BASE     0x40018000U
//#define AHB2_PERIPH_BASE

/*
 * Base addr of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASE_ADDR      (APB2_PERIPH_BASE + 0x0800)
#define GPIOB_BASE_ADDR      (APB2_PERIPH_BASE + 0x0C00)
#define GPIOC_BASE_ADDR      (APB2_PERIPH_BASE + 0x1000)
#define GPIOD_BASE_ADDR      (APB2_PERIPH_BASE + 0x1400)
#define GPIOE_BASE_ADDR      (APB2_PERIPH_BASE + 0x1800)
#define GPIOF_BASE_ADDR      (APB2_PERIPH_BASE + 0x1C00)
#define GPIOG_BASE_ADDR      (APB2_PERIPH_BASE + 0x2000)

/*
 * Base addr of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASE_ADDR      (APB2_PERIPH_BASE + 0x0800)
#define GPIOB_BASE_ADDR      (APB2_PERIPH_BASE + 0x0C00)
#define GPIOC_BASE_ADDR      (APB2_PERIPH_BASE + 0x1000)
#define GPIOD_BASE_ADDR      (APB2_PERIPH_BASE + 0x1400)
#define GPIOE_BASE_ADDR      (APB2_PERIPH_BASE + 0x1800)
#define GPIOF_BASE_ADDR      (APB2_PERIPH_BASE + 0x1C00)
#define GPIOG_BASE_ADDR      (APB2_PERIPH_BASE + 0x2000)


#endif /* INC_STM32F103XX_H_ */

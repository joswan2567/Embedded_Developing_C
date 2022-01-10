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
#define FLASH_BASE_ADDR           0x08000000U     /*base address flash bluepill*/
#define SRAM1_BASE_ADDR           0x20000000U     /*base address sram bluepill*/
//#define SRAM2_BASE_ADDR         0x20000000U     /*base address sram bluepill*/
#define ROM_BASE_ADDR             0x1FFFF000U     /*base address system memory bluepill*/
#define SRAM                      SRAM1_BASE_ADDR

/*
 * AHBx and APBX Bus Peripheral base addr
 */

#define PERIPH_BASE_ADDR          0x40000000U
#define APB1_PERIPH_BASE_ADDR     PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR     0x40010000U
#define AHB1_PERIPH_BASE_ADDR     0x40018000U
//#define AHB2_PERIPH_BASE

/*
 * Base addr of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define RCC_BASE_ADDR             0x40021000U

#define USB_OTG_FS_BASE_ADDR      0x50000000U

#define CRC_BASE_ADDR             0x40023000U

#define DMA1_BASE_ADDR            0x40020000U
#define DMA2_BASE_ADDR            0x40020400U

#define SDIO_BASE_ADDR            AHB1_PERIPH_BASE_ADDR


/*
 * Base addr of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define DAC_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x7400)

#define BKP_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x6C00)

#define CAN1_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x6400)
#define CAN2_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x6800)

#define IWDG_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x3000)

#define I2C1_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x5800)

#define PWR_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x7000)

#define RTC_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x2800)

#define SPI2_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x3C00)

#define TIMER2_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x0000)
#define TIMER3_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x0400)
#define TIMER4_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x0800)
#define TIMER5_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x0C00)
#define TIMER6_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x1000)
#define TIMER7_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x1400)
#define TIMER12_BASE_ADDR    (APB1_PERIPH_BASE_ADDR + 0x1800)
#define TIMER13_BASE_ADDR    (APB1_PERIPH_BASE_ADDR + 0x1C00)
#define TIMER14_BASE_ADDR    (APB1_PERIPH_BASE_ADDR + 0x2000)

#define USART2_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR     (APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR      (APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR      (APB1_PERIPH_BASE_ADDR + 0x5000)

#define WWDG_BASE_ADDR       (APB1_PERIPH_BASE_ADDR + 0x2C00)

/*
 * Base addr of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define ADC1_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x2400)
#define ADC2_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x2800)
#define ADC3_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x3C00)

#define AFIO_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x0000)

#define EXTI_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x0400)

#define GPIOA_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x0800)
#define GPIOB_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x1000)
#define GPIOD_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x1400)
#define GPIOE_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x1800)
#define GPIOF_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOG_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x2000)

#define SPI1_BASE_ADDR       (APB2_PERIPH_BASE_ADDR + 0x3000)

#define TIMER1_BASE_ADDR     (APB2_PERIPH_BASE_ADDR + 0x2C00)
#define TIMER8_BASE_ADDR     (APB2_PERIPH_BASE_ADDR + 0x3400)
#define TIMER9_BASE_ADDR     (APB2_PERIPH_BASE_ADDR + 0x4C00)
#define TIMER10_BASE_ADDR    (APB2_PERIPH_BASE_ADDR + 0x5000)
#define TIMER11_BASE_ADDR    (APB2_PERIPH_BASE_ADDR + 0x5400)

#define USART1_BASE_ADDR     (APB2_PERIPH_BASE_ADDR + 0x3800)

#endif /* INC_STM32F103XX_H_ */

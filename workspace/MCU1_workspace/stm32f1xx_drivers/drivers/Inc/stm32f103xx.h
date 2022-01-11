/*
 * stm32f103xx.h
 *
 *  Created on: Jan 9, 2022
 *      Author: José Wanderson
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>

#define __vo volatile

/*
 *  base addr of FLASH and SRAM  memories
 */
#define FLASH_BASE_ADDR           0x08000000U     /*base address flash bluepill*/
#define SRAM1_BASE_ADDR           0x20000000U     /*base address sram bluepill*/
#define ROM_BASE_ADDR             0x1FFFF000U     /*base address system memory bluepill*/
#define SRAM                      SRAM1_BASE_ADDR

/*
 * AHBx and APBX Bus Peripheral base addr
 */

#define PERIPH_BASE_ADDR          0x40000000U
#define APB1_PERIPH_BASE_ADDR     PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR     0x40010000U
#define AHB1_PERIPH_BASE_ADDR     0x40018000U

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


/******* Peripheral Register Definition Structures *********/

/*
 * Note : Registers of a peripheral are specific to MCU
 */

typedef struct{
	__vo uint32_t CRL;                          /*!< Port configuration register low,       Addr offset: 0x00  */
	__vo uint32_t CRH;                          /*!< Port configuration register high,      Addr offset: 0x04  */
	__vo uint32_t IDR;                          /*!< Port input data register,              Addr offset: 0x08  */
	__vo uint32_t ODR;                          /*!< Port output data register,             Addr offset: 0x0C  */
	__vo uint32_t BSRR;                         /*!< Port bit set/reset register,           Addr offset: 0x10  */
	__vo uint32_t BRR;                          /*!< Port bit reset register,               Addr offset: 0x14  */
	__vo uint32_t LCKR;                         /*!< Port configuration lock register,      Addr offset: 0x18  */

}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;                           /*!< Clock control register,                Addr offset: 0x00  */
	__vo uint32_t CFGR;                         /*!< Clock configuration register,          Addr offset: 0x04  */
	__vo uint32_t CIR;                          /*!< Clock interrupt register,              Addr offset: 0x08  */
	__vo uint32_t APB2RSTR;                     /*!< APB2 peripheral reset register,        Addr offset: 0x0C  */
	__vo uint32_t APB1RSTR;                     /*!< APB1 peripheral reset register,        Addr offset: 0x10  */
	__vo uint32_t AHBENR;                       /*!< AHB peripheral clock enable register,  Addr offset: 0x14  */
	__vo uint32_t APB2ENR;                      /*!< APB2 peripheral clock enable register, Addr offset: 0x18  */
	__vo uint32_t APB1ENR;                      /*!< APB1 peripheral clock enable register, Addr offset: 0x1C  */
	__vo uint32_t BDCR;                         /*!< Backup domain control register,        Addr offset: 0x20  */
	__vo uint32_t CSR;                          /*!< Control/status register,               Addr offset: 0x24  */

}RCC_RegDef_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */

#define GPIOA              ((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB              ((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC              ((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD              ((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE              ((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF              ((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG              ((GPIO_RegDef_t*) GPIOG_BASE_ADDR)

#define RCC                ((RCC_RegDef_t*) RCC_BASE_ADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 2 ) )             /*!<  GPIO port A clock enabled */
#define GPIOB_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 3 ) )             /*!<  GPIO port B clock enabled */
#define GPIOC_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 4 ) )             /*!<  GPIO port C clock enabled */
#define GPIOD_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 5 ) )             /*!<  GPIO port D clock enabled */
#define GPIOE_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 6 ) )             /*!<  GPIO port E clock enabled */
#define GPIOF_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 7 ) )             /*!<  GPIO port F clock enabled */
#define GPIOG_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 8 ) )             /*!<  GPIO port G clock enabled */

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 2 ) )             /*!<  GPIO port A clock disabled */
#define GPIOB_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 3 ) )             /*!<  GPIO port B clock disabled */
#define GPIOC_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 4 ) )             /*!<  GPIO port C clock disabled */
#define GPIOD_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 5 ) )             /*!<  GPIO port D clock disabled */
#define GPIOE_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 6 ) )             /*!<  GPIO port E clock disabled */
#define GPIOF_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 7 ) )             /*!<  GPIO port F clock disabled */
#define GPIOG_PCLK_DI()            ( RCC->APB2ENR &= ~( 1 << 8 ) )             /*!<  GPIO port G clock disabled */

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()             ( RCC->APB1ENR |= ( 1 << 21 ) )             /*!<  I2C1 clock enabled */
#define I2C2_PCLK_EN()             ( RCC->APB1ENR |= ( 1 << 22 ) )             /*!<  I2C2 clock enabled */

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()             ( RCC->APB1ENR &= ~( 1 << 21 ) )            /*!<  I2C1 clock disabled */
#define I2C2_PCLK_DI()             ( RCC->APB1ENR &= ~( 1 << 22 ) )            /*!<  I2C2 clock disabled */

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()             ( RCC->APB2ENR |= ( 1 << 12 ) )             /*!<  SPI1 clock enabled */
#define SPI2_PCLK_EN()             ( RCC->APB1ENR |= ( 1 << 14 ) )             /*!<  SPI2 clock enabled */
#define SPI3_PCLK_EN()             ( RCC->APB1ENR |= ( 1 << 15 ) )             /*!<  SPI3 clock enabled */
/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()             ( RCC->APB2ENR &= ~( 1 << 12 ) )            /*!<  SPI1 clock disabled */
#define SPI2_PCLK_DI()             ( RCC->APB1ENR &= ~( 1 << 14 ) )            /*!<  SPI2 clock disabled */
#define SPI3_PCLK_DI()             ( RCC->APB1ENR &= ~( 1 << 15 ) )            /*!<  SPI3 clock disabled */

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()           ( RCC->APB2ENR |= ( 1 << 14 ) )             /*!<  USART1 clock enabled */
#define USART2_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 17 ) )             /*!<  USART2 clock enabled */
#define USART3_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 18 ) )             /*!<  USART3 clock enabled */
#define UART4_PCLK_EN()            ( RCC->APB1ENR |= ( 1 << 19 ) )             /*!<  UART4 clock enabled */
#define UART5_PCLK_EN()            ( RCC->APB1ENR |= ( 1 << 20 ) )             /*!<  UART5 clock enabled */


/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()           ( RCC->APB2ENR &= ~( 1 << 14 ) )             /*!<  USART1 clock disabled */
#define USART2_PCLK_DI()           ( RCC->APB1ENR &= ~( 1 << 17 ) )             /*!<  USART2 clock disabled */
#define USART3_PCLK_DI()           ( RCC->APB1ENR &= ~( 1 << 18 ) )             /*!<  USART3 clock disabled */
#define UART4_PCLK_DI()            ( RCC->APB1ENR &= ~( 1 << 19 ) )             /*!<  UART4 clock disabled */
#define UART5_PCLK_DI()            ( RCC->APB1ENR &= ~( 1 << 20 ) )             /*!<  UART5 clock disabled */


/*
 * Generics Macros
 */

#define ENABLE    			1
#define DISABLE   			0
#define SET       			ENABLE
#define RESET     			DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET  	RESET

#endif /* INC_STM32F103XX_H_ */

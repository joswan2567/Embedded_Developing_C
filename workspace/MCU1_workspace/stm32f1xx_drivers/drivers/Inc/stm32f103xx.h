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

/******** Start Processor Specific Details **************
 *   													*
 *   ARM Cortex Mx Processor NVIC ISERx register addr	*
 *   													*
 ******************************************************/
#define NVIC_ISER0               ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1               ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2               ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3               ((__vo uint32_t*)0xE000E10C)

/*
 *   ARM Cortex Mx Processor NVIC ICERx register addr
 */
#define NVIC_ICER0               ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1               ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2               ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3               ((__vo uint32_t*)0xE000E18C)

/*
 *   ARM Cortex Mx Processor Priority Register Addr Calculation
 */
#define NVIC_PR_BASE_ADDR        ((__vo uint32_t*)0xE000E400)

/*
 *   ARM Cortex Mx Processor Number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED   4

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

/*
 * peripheral register definition structure for GPIOx
 */
typedef struct{
	__vo uint32_t CR[2];                          /*!< Port configuration register low [0] and high [1],       Addr offset: 0x00  */
	__vo uint32_t IDR;                          /*!< Port input data register,              Addr offset: 0x08  */
	__vo uint32_t ODR;                          /*!< Port output data register,             Addr offset: 0x0C  */
	__vo uint32_t BSRR;                         /*!< Port bit set/reset register,           Addr offset: 0x10  */
	__vo uint32_t BRR;                          /*!< Port bit reset register,               Addr offset: 0x14  */
	__vo uint32_t LCKR;                         /*!< Port configuration lock register,      Addr offset: 0x18  */

}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
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
 * peripheral register definition structure for EXTI
 */
typedef struct{
	__vo uint32_t IMR;                          /*!< Interrupt mask register,               Addr offset: 0x00  */
	__vo uint32_t EMR;                          /*!< Event mask register,                   Addr offset: 0x04  */
	__vo uint32_t RTSR;                         /*!< Rising trigger selection register,     Addr offset: 0x08  */
	__vo uint32_t FTSR;                         /*!< Falling trigger selection register,    Addr offset: 0x0C  */
	__vo uint32_t SWIER;                        /*!< Software interrupt event register,     Addr offset: 0x10  */
	__vo uint32_t PR;                           /*!< Pending register,                      Addr offset: 0x14  */

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for AFIO
 */
typedef struct{
	__vo uint32_t EVCR;                          /*!< Event control register ,               			 Addr offset: 0x00  */
	__vo uint32_t MAPR;                          /*!< AF remap and debug I/O configuration register,     Addr offset: 0x04  */
	__vo uint32_t EXTICR[4];                     /*!< External interrupt configuration register,		 Addr offset: 0x08  */
	__vo uint32_t MAPR2;                         /*!< AF remap and debug I/O configuration register2,    Addr offset: 0x1C  */

}AFIO_RegDef_t;

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

#define EXTI			   ((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define AFIO			   ((AFIO_RegDef_t*) AFIO_BASE_ADDR)

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
 * returns port code for given GPIOx base addr
 */

#define GPIO_BASE_ADDR_TO_CODE(x)       ((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 : -1)
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
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOB_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOC_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOD_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOE_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOF_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7)); }while(0)
#define GPIOG_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8)); }while(0)

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
 * Clock Enable Macros for AFIO peripherals
 */

#define AFIO_PCLK_EN()            ( RCC->APB2ENR |= ( 1 << 0 ) )             /*!<  GPIO port A clock enabled */


/*
 * IRQ(Interrupt Request) Numbers of STM32F103x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10 	40

/*
 * IRQ(Interrupt Request) Numbers of Priority possible
 */

#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*
 * Generics Macros
 */

#define ENABLE    			1
#define DISABLE   			0
#define SET       			ENABLE
#define RESET     			DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET  	RESET

#include "stm32f103xx_gpio_driver.h"

#endif /* INC_STM32F103XX_H_ */

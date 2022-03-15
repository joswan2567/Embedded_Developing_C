/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t payLoad[10];
	uint32_t len;
}cmd_t;

typedef enum{
	sMainMenu = 0,
	sLedEffect,
	sRtcMenu,
	sRtcTimeConfig,
	sRtcDateConfig,
	sRtcReport,
}state_t;

extern TaskHandle_t led_handler;
extern TaskHandle_t menu_handler;
extern TaskHandle_t rtc_handler;

extern TaskHandle_t print_handler;
extern TaskHandle_t cmd_handler;
extern TaskHandle_t l_handler;

extern QueueHandle_t InputData_Queue;
extern QueueHandle_t Print_Queue;

extern RTC_HandleTypeDef hrtc;

extern UART_HandleTypeDef huart1;

extern state_t curr_state;

extern TimerHandle_t h_led_timer[4];
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void menu_task(void *pvParameters);
void led_task(void *pvParameters);
void rtc_task(void *pvParameters);
void print_task(void *pvParameters);
void cmd_task(void *pvParameters);

void process_cmd(cmd_t *cmd);
int extract_cmd(cmd_t *cmd);


void led_effect_stop(void);
void led_effect(int opc);

void LED_effect1(void);
void LED_effect2(void);
void LED_effect3(void);
void LED_effect4(void);

void show_time_date(void);
void show_time_date_itm(void);
void rtc_configure_time(RTC_TimeTypeDef *time);
void rtc_configure_date(RTC_DateTypeDef *date);
int validate_rtc_information(RTC_TimeTypeDef *time , RTC_DateTypeDef *date);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Green_Pin GPIO_PIN_12
#define LED_Green_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_13
#define LED_Red_GPIO_Port GPIOB
#define LED_Yellow_Pin GPIO_PIN_14
#define LED_Yellow_GPIO_Port GPIOB
#define LED_Blue_Pin GPIO_PIN_15
#define LED_Blue_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

TaskHandle_t ledG_handle, ledR_handle, ledY_handle, ledB_handle, btn_handle;
TaskHandle_t volatile nxt_task_handle = NULL;
BaseType_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ledRed_handler(void *pvParameters);
void ledBlue_handler(void *pvParameters);
void ledYellow_handler(void *pvParameters);
void ledGreen_handler(void *pvParameters);

void button_handler(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */
	DWT_CTRL |= (1 << 0);

	status = xTaskCreate(ledGreen_handler, "LED_Green_Task", 50, NULL, 4, &ledG_handle);

	configASSERT(status == pdPASS);

	nxt_task_handle = ledG_handle;

	status = xTaskCreate(ledYellow_handler, "LED_Yellow_Task", 50, NULL, 3, &ledY_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(ledRed_handler, "LED_Red_Task", 50, NULL, 2, &ledR_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(ledBlue_handler, "LED_Blue_Task", 50, NULL, 1, &ledB_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(button_handler, "Button_Task", 50, NULL, 5, &btn_handle);

	configASSERT(status == pdPASS);

	vTaskStartScheduler();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_Green_Pin|LED_Red_Pin|LED_Yellow_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_Green_Pin LED_Red_Pin LED_Yellow_Pin LED_Blue_Pin */
	GPIO_InitStruct.Pin = LED_Green_Pin|LED_Red_Pin|LED_Yellow_Pin|LED_Blue_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ledGreen_handler(void *pvParameters){

	//TickType_t delay = pdMS_TO_TICKS(1200), initTask = xTaskGetTickCount();
	BaseType_t status;
	while(1){

		HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
		//HAL_Delay(1000);
		//vTaskDelay(delay);
		//vTaskDelayUntil(&initTask, delay);
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1200));
		if(status == pdTRUE){
			vTaskSuspendAll();
			nxt_task_handle = ledR_handle;
			xTaskResumeAll();
			HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
			vTaskDelete(NULL);
		}
	}

}
void ledYellow_handler(void *pvParameters){

	//TickType_t delay = pdMS_TO_TICKS(900), initTask = xTaskGetTickCount();
	BaseType_t status;
	while(1){

		HAL_GPIO_TogglePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin);
		//HAL_Delay(800);
		//vTaskDelay(delay);
		//vTaskDelayUntil(&initTask, delay);
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(900));
		if(status == pdTRUE){
			vTaskSuspendAll();
			nxt_task_handle = ledB_handle;
			xTaskResumeAll();
			HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_SET);
			vTaskDelete(NULL);
		}
	}
}
void ledRed_handler(void *pvParameters){

	//TickType_t delay = pdMS_TO_TICKS(600), initTask = xTaskGetTickCount();
	BaseType_t status;
	while(1){
		//SEGGER_SYSVIEW_PrintfTarget("teste");

		HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
		//HAL_Delay(400);
		//vTaskDelay(delay);
		//vTaskDelayUntil(&initTask, delay);
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(600));
		if(status == pdTRUE){
			vTaskSuspendAll();
			nxt_task_handle = ledY_handle;
			xTaskResumeAll();
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
			vTaskDelete(NULL);
		}
	}
}
void ledBlue_handler(void *pvParameters){

	//TickType_t delay = pdMS_TO_TICKS(300), initTask = xTaskGetTickCount();
	BaseType_t status;
	while(1){
		//SEGGER_SYSVIEW_PrintfTarget("teste");
		HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
		//HAL_Delay(400);
		//vTaskDelay(delay);
		//vTaskDelayUntil(&initTask, delay);
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(300));
		if(status == pdTRUE){
			vTaskSuspendAll();
			nxt_task_handle = NULL;
			xTaskResumeAll();
			HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
			vTaskDelete(btn_handle);
			vTaskDelete(NULL);
		}
	}
}
void button_handler(void *pvParameters){

	uint8_t btn_read = 0;
	uint8_t prev_read = 1;
	while(1){

		btn_read = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

		if(btn_read){
			if( ! prev_read)
				xTaskNotify(nxt_task_handle, 0, eNoAction);
		}
		prev_read = btn_read;
		vTaskDelay(pdMS_TO_TICKS(10));

	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


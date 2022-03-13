/*
 * led_effect.c
 *
 *  Created on: Mar 6, 2022
 *      Author: José Wanderson
 */

#include "main.h"

void turn_off_all_leds(void){

	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
}

void turn_on_all_leds(void){

	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
}

void turn_on_odd_leds(void)
{
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
}


void turn_on_even_leds(void)
{
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
}

void led_effect_stop(void){
	for(int i = -1 ; i++< 4;)
		xTimerStop(h_led_timer[i], portMAX_DELAY);
	turn_off_all_leds();
}

void led_effect(int opc){

	led_effect_stop();
	xTimerStart(h_led_timer[opc - 1], portMAX_DELAY);
}

void LED_control( int value )
{
  for(int i = 0 ; i < 4 ; i++)
	  HAL_GPIO_WritePin(LED_Green_GPIO_Port, (LED_Green_Pin << i), ((value >> i)& 0x1));
}

void LED_effect1(void){

	static int flag = 1;
	(flag ^=1) ? turn_off_all_leds() : turn_on_all_leds();
}

void LED_effect2(void){

	static int flag = 1;
	(flag ^=1) ? turn_on_even_leds() : turn_on_odd_leds();
}

void LED_effect3(void){

	static int i = 0;
	LED_control(0x01 << ( i++ % 4));
}

void LED_effect4(void){

	static int i = 0;
	LED_control( 0x08 >> (i++ % 4));
}

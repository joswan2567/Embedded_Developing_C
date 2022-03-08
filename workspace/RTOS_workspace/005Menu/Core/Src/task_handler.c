/*
 * task_handler.c
 *
 *  Created on: Mar 6, 2022
 *      Author: José Wanderson
 */

#include "task_handler.h"

void menu_task(void *pvParameters){
	while(1){

	}
}
void led_task(void *pvParameters){
	while(1){

	}
}
void rtc_task(void *pvParameters){
	while(1){

	}
}
void print_task(void *pvParameters){
	while(1){

	}
}
void cmd_task(void *pvParameters){

	BaseType_t ret;
	cmd_t cmd;
	while(1){

		ret = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

		if(ret == pdTRUE){
			process_cmd(&cmd);
		}
	}
}

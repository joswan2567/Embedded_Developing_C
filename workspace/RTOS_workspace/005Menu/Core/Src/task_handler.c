/*
 * task_handler.c
 *
 *  Created on: Mar 6, 2022
 *      Author: José Wanderson
 */

#include "task_handler.h"

const char *msg_inv = "\n*** Invalid Option! ***\n";

void menu_task(void *pvParameters){

	uint32_t cmd_addr;

	cmd_t *cmd;

	int option;

	const char* msg_menu = "========================\n"
						   "|      	   Menu        |\n"
						   "========================\n"
							 "LED Effect    ----> 0\n"
							 "Date and time ----> 1\n"
							 "Exit			----> 2\n\n"
							 "Enter your choice here : ";

	while(1){
		xQueueSend(Print_Queue, &msg_menu, portMAX_DELAY);

		xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);

		cmd = (cmd_t*)cmd_addr;

		if(cmd->len == 1){
			option = cmd->payLoad[0] - 48; //converter ASCII in number
			switch(option){
			case 0:
				curr_state = sLedEffect;
				xTaskNotify(led_handler, 0, eNoAction);
				break;
			case 1:
				curr_state = sRtcMenu;
				xTaskNotify(rtc_handler, 0, eNoAction);
				break;
			case 2:
				break;
			default:
				xQueueSend(InputData_Queue, &msg_inv, portMAX_DELAY);
				continue;
			}
		}
		else{
			xQueueSend(InputData_Queue, &msg_inv, portMAX_DELAY);
			continue;

		}
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
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

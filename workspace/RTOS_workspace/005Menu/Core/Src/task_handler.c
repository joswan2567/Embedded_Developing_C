/*
 * task_handler.c
 *
 *  Created on: Mar 6, 2022
 *      Author: José Wanderson
 */

#include "main.h"

void process_cmd(cmd_t *cmd);
int extract_cmd(cmd_t *cmd);

const char *msg_inv = "\n*** Invalid Option! ***\n";

void menu_task(void *pvParameters){

	uint32_t cmd_addr;

	cmd_t *cmd;

	int option;

	const char* msg_menu = "\n========================\n"
			"|         Menu         |\n"
			"========================\n"
			"LED effect    ----> 0\n"
			"Date and time ----> 1\n"
			"Exit          ----> 2\n"
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
				xTaskNotify(l_handler, 0, eNoAction);
				break;
			case 1:
				curr_state = sRtcMenu;
				xTaskNotify(rtc_handler, 0, eNoAction);
				break;
			case 2:
				break;
			default:
				xQueueSend(Print_Queue, &msg_inv, portMAX_DELAY);
				continue;
			}
		}
		else{
			xQueueSend(Print_Queue, &msg_inv, portMAX_DELAY);
			continue;

		}
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	}
}
void led_task(void *pvParameters){

	uint32_t cmd_addr;
	cmd_t *cmd;
	const char* msg_led = "\n========================\n"
			"|      LED Effect     |\n"
			"========================\n"
			"(none,e1,e2,e3,e4)\n"
			"Enter your choice here : ";
	while(1){
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		xQueueSend(Print_Queue, &msg_led, portMAX_DELAY);
		xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);

		cmd = (cmd_t*) cmd_addr;

		if(cmd->len <= 4){
			if		( ! strcmp((char*)cmd->payLoad, "none")) led_effect_stop();
			else if ( ! strcmp((char*)cmd->payLoad, "e1")) led_effect(1);
			else if ( ! strcmp((char*)cmd->payLoad, "e2")) led_effect(2);
			else if ( ! strcmp((char*)cmd->payLoad, "e3")) led_effect(3);
			else if ( ! strcmp((char*)cmd->payLoad, "e4")) led_effect(4);
			else xQueueSend(Print_Queue, (void*)&msg_inv, portMAX_DELAY);
		}
		else
			xQueueSend(Print_Queue, (void*)&msg_inv, portMAX_DELAY);
		curr_state = sMainMenu;

		xTaskNotify(menu_handler, 0, eNoAction);
	}
}
void rtc_task(void *pvParameters){

	const char *t = "coe man";
	while(1){
		xTaskNotifyWait(0,0,NULL, portMAX_DELAY);
		xQueueSend(Print_Queue, &t, portMAX_DELAY);
		curr_state = sMainMenu;

		xTaskNotify(menu_handler, 0, eNoAction);

	}
}
void print_task(void *pvParameters){

	uint32_t *msg;
	while(1){
		xQueueReceive(Print_Queue, &msg, portMAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);
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

void process_cmd(cmd_t *cmd){

	extract_cmd(cmd);

	switch(curr_state){

	case sMainMenu:
		/*TODO: */
		xTaskNotify(menu_handler, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	case sLedEffect:
		/*TODO: */
		xTaskNotify(l_handler, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	case sRtcMenu:
		/*TODO: */
	case sRtcTimeConfig:
		/*TODO: */
	case sRtcDateConfig:
		/*TODO: */
	case sRtcReport:
		/*TODO: */
		xTaskNotify(rtc_handler, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	}
}

int extract_cmd(cmd_t *cmd){

	uint8_t item;
	BaseType_t status;

	status = uxQueueMessagesWaiting(InputData_Queue); // api que retorna o numero de elementos na fila
	if( ! status) return -1;
	uint8_t i = 0;

	do{
		status = xQueueReceive(InputData_Queue, &item, 0);
		if(status == pdTRUE) cmd->payLoad[i++] = item;
	}while(item != '\n');

	cmd->payLoad[i-1] = '\0';
	cmd->len = i-1;

	return 0;
}

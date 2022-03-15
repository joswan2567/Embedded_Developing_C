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

uint8_t getnumber(uint8_t *p , int len)
{

	int value ;

	if(len > 1)
	   value =  ( ((p[0]-48) * 10) + (p[1] - 48) );
	else
		value = p[0] - 48;

	return value;

}

void rtc_task(void *pvParameters){

	const char* msg_rtc1 = "\n========================\n"
			"|         RTC          |\n"
			"========================\n";

	const char* msg_rtc2 = "\nConfigure Time            ----> 0\n"
			"Configure Date            ----> 1\n"
			"Enable reporting          ----> 2\n"
			"Exit                      ----> 4\n"
			"Enter your choice here : ";


	const char *msg_rtc_hh = "\nEnter hour(1-12):";
	const char *msg_rtc_mm = "\nEnter minutes(0-59):";
	const char *msg_rtc_ss = "\nEnter seconds(0-59):";

	const char *msg_rtc_dd  = "\nEnter date(1-31):";
	const char *msg_rtc_mo  ="\nEnter month(1-12):";
	const char *msg_rtc_dow  = "\nEnter day(1-7 sun:1):";
	const char *msg_rtc_yr  = "\nEnter year(0-99):";

	const char *msg_conf = "\n*** Configuration successful! ***\n";
	const char *msg_rtc_report = "\nEnable time&date reporting(y/n)?: ";


	uint32_t cmd_addr;
	cmd_t *cmd;


	static int rtc_state = 0;
	int menu_code;

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

#define HH_CONFIG 		0
#define MM_CONFIG 		1
#define SS_CONFIG 		2

#define DATE_CONFIG 	0
#define MONTH_CONFIG 	1
#define YEAR_CONFIG 	2
#define DAY_CONFIG 		3


	while(1){
		/*TODO: Notify wait (wait till someone notifies)		 */
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		/*TODO : Print the menu and show current date and time information */
		xQueueSend(Print_Queue, &msg_rtc1, portMAX_DELAY);
		show_time_date();
		xQueueSend(Print_Queue, &msg_rtc2, portMAX_DELAY);
		//xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);

		while(curr_state != sMainMenu){

			/*TODO: Wait for command notification (Notify wait) */
			xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
			cmd = (cmd_t*) cmd_addr;

			switch(curr_state)
			{
			case sRtcMenu:{

				/*TODO: process RTC menu commands */
				if(cmd->len == 1)
				{
					menu_code = cmd->payLoad[0] - 48;
					switch(menu_code)
					{
					case 0:
						curr_state = sRtcTimeConfig;
						xQueueSend(Print_Queue,&msg_rtc_hh,portMAX_DELAY);
						break;
					case 1:
						curr_state = sRtcDateConfig;
						xQueueSend(Print_Queue,&msg_rtc_dd,portMAX_DELAY);
						break;
					case 2 :
						curr_state = sRtcReport;
						xQueueSend(Print_Queue,&msg_rtc_report,portMAX_DELAY);
						break;
					case 3 :
						curr_state = sMainMenu;
						break;
					default:
						curr_state = sMainMenu;
						xQueueSend(Print_Queue,&msg_inv,portMAX_DELAY);
					}
				}
				else{
					curr_state = sMainMenu;
					xQueueSend(Print_Queue, &msg_inv, portMAX_DELAY);
				}

				break;}

			case sRtcTimeConfig:{
				/*TODO : get hh, mm, ss infor and configure RTC */
				switch(rtc_state)
				{
				case HH_CONFIG:{
					uint8_t hour = getnumber(cmd->payLoad , cmd->len);
					time.Hours = hour;
					rtc_state = MM_CONFIG;
					xQueueSend(Print_Queue,&msg_rtc_mm,portMAX_DELAY);
					break;}
				case MM_CONFIG:{
					uint8_t min = getnumber(cmd->payLoad , cmd->len);
					time.Minutes = min;
					rtc_state = SS_CONFIG;
					xQueueSend(Print_Queue,&msg_rtc_ss,portMAX_DELAY);
					break;}
				case SS_CONFIG:{
					uint8_t sec = getnumber(cmd->payLoad , cmd->len);
					time.Seconds = sec;
					if(!validate_rtc_information(&time,NULL))
					{
						rtc_configure_time(&time);
						xQueueSend(Print_Queue,&msg_conf,portMAX_DELAY);
						show_time_date();
					}else
						xQueueSend(Print_Queue,&msg_inv,portMAX_DELAY);

					curr_state = sMainMenu;
					rtc_state = 0;
					break;}
				}

				/*TODO: take care of invalid entries */
				break;}

			case sRtcDateConfig:{

				/*TODO : get date, month, day , year info and configure RTC */

				/*TODO: take care of invalid entries */

				break;}

			case sRtcReport:{
				/*TODO: enable or disable RTC current time reporting over ITM printf */
				break;}

			}// switch end

		} //while end

		/*TODO : Notify menu task */
		curr_state = sMainMenu;
		xTaskNotify(menu_handler, 0, eNoAction);


	}//while super loop end
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

/*
 * task_handler.h
 *
 *  Created on: Mar 6, 2022
 *      Author: José Wanderson
 */

#ifndef INC_TASK_HANDLER_H_
#define INC_TASK_HANDLER_H_

#include "main.h"

void menu_task(void *pvParameters);
void led_task(void *pvParameters);
void rtc_task(void *pvParameters);
void print_task(void *pvParameters);
void cmd_task(void *pvParameters);

#endif /* INC_TASK_HANDLER_H_ */

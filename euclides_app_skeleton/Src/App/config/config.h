/*
 * config.h
 *
 *  Created on: 28.06.2017
 *      Author: skrzyraf
 */

#ifndef APP_CONFIG_CONFIG_H_
#define APP_CONFIG_CONFIG_H_

#include "cmsis_os.h"

#define MAX_WORKING_TASKS        3

/* List of working tasks */
osThreadId working_tasks_list[MAX_WORKING_TASKS];

void KeepOnlyThisTask(osThreadId keeping_id);

#endif /* APP_CONFIG_CONFIG_H_ */

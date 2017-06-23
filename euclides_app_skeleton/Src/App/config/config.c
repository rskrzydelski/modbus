/*
 * config.c
 *
 *  Created on: 28.06.2017
 *      Author: skrzyraf
 */

#include "config.h"



void KeepOnlyThisTask(osThreadId keeping_id)
{
	int i;

	for (i = 0; i < MAX_WORKING_TASKS; i++) {

		if (working_tasks_list[i] == keeping_id) {
			continue;
		}

		osThreadSuspend(working_tasks_list[i]);
	}
}

/*
 * safety_switch_task.h
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */

#ifndef APP_MEASURE_INCLUDE_SAFETY_SWITCH_TASK_H_
#define APP_MEASURE_INCLUDE_SAFETY_SWITCH_TASK_H_

#include "cmsis_os.h"

osThreadId safety_switch_task_handle;

void safety_switch_task(void const * argumen);

#endif /* APP_MEASURE_INCLUDE_SAFETY_SWITCH_TASK_H_ */

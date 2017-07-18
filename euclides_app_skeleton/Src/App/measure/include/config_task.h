/*
 * config_task.h
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */

#ifndef APP_MEASURE_INCLUDE_CONFIG_TASK_H_
#define APP_MEASURE_INCLUDE_CONFIG_TASK_H_

#include "cmsis_os.h"

osThreadId config_task_handle;

void config_task(void const * argumen);

/* Configuration variables */
volatile uint16_t time_overflow_cnf;        /* Time in [ms] how long have to be last until last edge on encoder that we treat as stop machine */

#endif /* APP_MEASURE_INCLUDE_CONFIG_TASK_H_ */

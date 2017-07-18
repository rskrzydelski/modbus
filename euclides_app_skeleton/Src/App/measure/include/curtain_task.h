/*
 * curtain_task.h
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */

#ifndef APP_MEASURE_INCLUDE_CURTAIN_TASK_H_
#define APP_MEASURE_INCLUDE_CURTAIN_TASK_H_

#include "cmsis_os.h"

osThreadId curtain_task_handle;

void curtain_task(void const * argumen);

#endif /* APP_MEASURE_INCLUDE_CURTAIN_TASK_H_ */

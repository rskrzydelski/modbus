/*
 * safety_switch_task.c
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */
#include "safety_switch_task.h"
#include "modbus_app.h"

void safety_switch_task(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
	  data_to_master[data_to_master_2]++;
  }
}

/*
 * config_task.c
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */
#include "config_task.h"
#include "modbus_app.h"

void config_task(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	  time_overflow_cnf = data_from_master[time_overflow_idx];
  }
}

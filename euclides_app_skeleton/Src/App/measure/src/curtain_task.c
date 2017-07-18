/*
 * curtain_task.c
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */
#include "curtain_task.h"
#include "modbus_app.h"

void curtain_task(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
	  data_to_master[data_to_master_3]++;
  }
}

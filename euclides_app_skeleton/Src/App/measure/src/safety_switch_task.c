/*
 * safety_switch_task.c
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */
#include "safety_switch_task.h"
#include "modbus_app.h"
#include "euclides.h"

/* Initialize all stuff regarding to measurement (variables, interrupt, etc.) */
static void init_procedures(void)
{

}

/* This function is called during terminate procedure */
static void clean_up(void)
{

}

void safety_switch_task(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	  data_to_master[data_to_master_2]++;

      /* Handle of terminate button */
      if (set_coils_by_master[momentary_sw_g1] == SW8_G1_TERMINATE_PROCEDURE) {
              terminate_procedure(clean_up, osThreadGetId());
      }
  }
}

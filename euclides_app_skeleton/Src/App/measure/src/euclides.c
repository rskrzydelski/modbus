/*
 * euclides.c
 *
 *  Created on: 23.06.2017
 *      Author: skrzyraf
 */
#include "euclides.h"

void terminate_procedure(clean_up_cb_t clean_up_cb, osThreadId thread_id)
{
        if (clean_up_cb != NULL) {
		        clean_up_cb();
        }

        CLEAR_ALL_ST_G1;
        CLEAR_ALL_ST_G2;
        RST_BUTTON_G1(SW8_G1_TERMINATE_PROCEDURE);
        /* TODO: Handle NULL case */
        osThreadSuspend(thread_id);
}

void stop_machine_signal_handle(void)
{
        /* This is timer for machine stop time measurement , resolution 1 us, set to 0 because we've just gave stop signal */
        MACHINE_STOP_TIME = 0;

        /* Actual encoder position */
        machine_stop_signal_position = MACHINE_POSITION;

        /* Actual velocity */

}

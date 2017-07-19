/*
 * curtain_task.c
 *
 *  Created on: 18.07.2017
 *      Author: skrzyraf
 */
#include "stm32f4xx_hal.h"

#include "curtain_task.h"
#include "modbus_app.h"
#include "config_task.h"
#include "euclides.h"

#define TOO_LONG_SIGNAL        10
#define TOO_SHORT_SIGNAL       1

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;

enum {
    initialization, choice_mode, looking_for_start_position, wait_for_movement, measurement_procedure,
	wait_for_done, measurement_done, measurement_fault,
    CURTAIN_TASK_STEPS_NUM
};

void xor_no_signal_callback(void const *argument)
{
        SET_STATUS_G1(ST_G1_XOR_FAULT_NO_SIGNAL);
        step = measurement_fault;
}

void timer_on_delay_callback(void const *argument)
{
        RST_STATUS_G1(ST_G1_MEAS_IN_PROGRESS);
        RST_STATUS_G1(ST_G1_IS_MOVING);
        SET_STATUS_G1(ST_G1_MEAS_IS_DONE);
        step = measurement_done;
}

/* Initialize all stuff regarding to measurement (variables, interrupt, etc.) */
static void init_procedures(void)
{
        /* Start encoder mode */
        HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

        /* Clear direction to 0 (direction up) */
        TIM1->CR1 &= ~(TIM_CR1_DIR);

        xor_no_signal_def.ptimer = xor_no_signal_callback;
        xor_no_signal_id = osTimerCreate (&xor_no_signal_def, osTimerOnce, NULL);

        timer_on_delay_def.ptimer = timer_on_delay_callback;
        timer_on_delay_id = osTimerCreate (&timer_on_delay_def, osTimerOnce, NULL);
}

/* This function is called during terminate procedure */
static void clean_up(void)
{
        /* Disable encoder interrupt */
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

        /* Disable encoder mode */
        HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);

        /* Disable ICP Interrupt - measure of actuator pulse */
        __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_CC1);

        /* Disable timer on delay */
        HAL_TIM_Base_Stop_IT(&htim5);

        /* Clear start actuator signal */
        HAL_GPIO_WritePin(ACTUATOR_START_GPIO_Port, ACTUATOR_START_Pin, GPIO_PIN_RESET);

        /* Clean all software timers */
        osTimerStop (xor_no_signal_id);
        osTimerDelete (xor_no_signal_id);

        osTimerStop (timer_on_delay_id);
        osTimerDelete (timer_on_delay_id);

        /* Clear measurement variables */
        actuator_pulse = 0;

        step = 0;
}

void curtain_task(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	   /* DEBUG: to remove */
	   data_to_master[data_to_master_8] = step;

        if (step == initialization) {
                /* Initialize all variables, interrupts, etc. */
                init_procedures();
                step = choice_mode;
        }

        if (step == choice_mode) {
                switch (set_coils_by_master[momentary_sw_g2]) {
                        case SW1_G2_MANUAL_MODE:
                                SET_STATUS_G1(ST_G1_MANUAL_MODE);
                                RST_BUTTON_G2(SW1_G2_MANUAL_MODE);

                                step = looking_for_start_position;
                                break;

                         case SW2_G2_AUTOMATIC_MODE:
                                  /* TODO: Future implementation */
                                  RST_BUTTON_G2(SW2_G2_AUTOMATIC_MODE);
                                break;
                }
        }

        if (step == looking_for_start_position) {

                /* MSG: Ustaw maszynê w miejscu zerowym (pocz¹tek ruchu roboczego maszyny) */
                if (set_coils_by_master[momentary_sw_g2] == SW3_G2_POSITION_IS_ZERO) {
                        /* Clear pending external interrupts */
                        __HAL_GPIO_EXTI_CLEAR_IT(L_ENCODER_IND_A_Pin);
                        __HAL_GPIO_EXTI_CLEAR_IT(L_ENCODER_IND_B_Pin);

                        /* Enable external interrupt - encoder edges */
                        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

                        /* Set encoder to zero position */
                        TIM1->CNT = 0;

                        step = wait_for_movement;

                        RST_BUTTON_G2(SW3_G2_POSITION_IS_ZERO);
        	    }
        }

        if (step == wait_for_movement) {
                /* XXX: Critical section - from now the action is in critical_interrupt.c HAL_GPIO_EXTI_Callback.
                 * In this procedure program looking for machine movement, when it is detect ST_G1_IS_MOVING status is set.
                 * */
                if (STATUS_G1_IS_SET(ST_G1_IS_MOVING)) {
                        step = measurement_procedure;
                }
        }

        if (step == measurement_procedure) {
                /* Handle stop machine button - manual mode */
                if (set_coils_by_master[momentary_sw_g2] == SW4_G2_STOP_MACHINE) {

                        /* Set status */
                        SET_STATUS_G1(ST_G1_MEAS_IN_PROGRESS);

                        /* Start Input Capture Interrupt - for measurement maximum fault of actuator */
            	        HAL_TIM_IC_Start_IT(&htim11, TIM_CHANNEL_1);

            	        /* Start fuse timer for XOR - 100 ms - the IC overflow tread after ~ 655 ms,
            	         * so value 100 ms protect as to overflow value of IC.
            	         *  */
            	        osTimerStart (xor_no_signal_id, 100);

            	        /* Start timer on delay - this timer is reset on every edge from encoder,
            	         * if time specify in parameter time_overflow_cnf elapsed then measurement is done
            	         *  */
            	        osTimerStart (timer_on_delay_id, time_overflow_cnf);

                        /* Start actuator - interrupt curtain */
                        HAL_GPIO_WritePin(ACTUATOR_START_GPIO_Port, ACTUATOR_START_Pin, GPIO_PIN_SET);
                        /* XXX: Critical section - from now we wait for IC - this is signal that we start
                         * measure stop time and distance
                         */

                        RST_BUTTON_G2(SW4_G2_STOP_MACHINE);

                        step = wait_for_done;
            }
        }

        if (step == wait_for_done) {
                /* XXX: Critical section - we wait for timer_on_delay_callback (is called when measurement is done)
                 * which sets ST_G1_MEAS_IS_DONE status and give as to measurement_done procedure.
                 */
        }

        if (step == measurement_done) {

                /* We take in to account following scenarios:
        	     *- we encounter only rising edge;                          Protect mechanism - Software timer: xor_no_signal_id
        	     *- we don't have any signal at all;                        Protect mechanism - Software timer: xor_no_signal_id
        	     *- the signal is too long - above TOO_LONG_SIGNAL;         Protect mechanism - Condition below
        	     *- the signal is strangely short - below TOO_SHORT_SIGNAL; Protect mechanism - Condition below
        	     *- detect overflow;                                        Protect mechanism - Software timer: xor_no_signal_id
        	     *  */
                 /* Calculation from ticks to ms */
                 xor_pulse_ms__ = (actuator_pulse * 10) / 1000;

        	     if (xor_pulse_ms__ > TOO_LONG_SIGNAL || xor_pulse_ms__ < TOO_SHORT_SIGNAL) {
                         SET_STATUS_G1(ST_G1_XOR_FAULT_WRONG_DURATION);
                         step = measurement_fault;
                         break; ???
        	     }

        	     /* Measurement data to modbus */
        	     data_to_master[xor_pulse] = xor_pulse_ms__;
        	     data_to_master[stop_machine_time] = stop_machine_time__;
        	     data_to_master[stop_distance] = stop_distance__;
        }

        /* If something going wrong */
        if (step == measurement_fault) {
                /* TODO: Now after any fault we start measurement procedure from scratch.
                 * Please implement various scenarios depend on the fault status */
                clean_up();
                step = initialization;
        }

        /* Handle of terminate button */
        if (set_coils_by_master[momentary_sw_g1] == SW8_G1_TERMINATE_PROCEDURE) {
                terminate_procedure(clean_up, osThreadGetId());
        }
  }
}

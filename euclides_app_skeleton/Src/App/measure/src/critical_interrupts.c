/*
 * critical_interrupts.c
 *
 *  Created on: 19.07.2017
 *      Author: skrzyraf
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "modbus_app.h"
#include "modbus.h"
#include "euclides.h"

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Start timer - 3.5 char (~ 2ms) */
	HAL_TIM_Base_Start_IT(&htim10);

    HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_SET);

	/* Put data into modbus parser */
	receive_modbus_message(data_in_item);

    /* Listen for input again */
    HAL_UART_Receive_DMA(&huart1, &data_in_item, 1);
}

/* This interrupt is called on every encoder edge */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        /* FIXME: Check from which pin interrupt is */
        /* If machine go at least 10 mm, then set ST_G1_IS_MOVING status - detect machine movement */
        if (TIM1->CNT > 200) {
                SET_STATUS_G1(ST_G1_IS_MOVING);
        }

        /* Information from encoder - every edge */
        if (DIR == LINEAR_ROLL_OUT) {
            if (GPIO_Pin == L_ENCODER_IND_A_Pin) {
            	/* Assign machine stop time */
            	stop_machine_time__ = MACHINE_STOP_TIME;
            	/* Assign machine stop distance */
            	stop_distance__ = (MACHINE_POSITION - machine_stop_signal_position);
            	/* Clear timer on delay */
//            	xTimerResetFromISR(timer_on_delay_id, osPriorityNormal + 1);
            }

            if (GPIO_Pin == L_ENCODER_IND_B_Pin) {
            	/* Assign machine stop time */
            	stop_machine_time__ = MACHINE_STOP_TIME;
            	/* Assign machine stop distance */
            	stop_distance__ = (MACHINE_POSITION - machine_stop_signal_position);
            	/* Clear timer on delay */
//            	xTimerResetFromISR(timer_on_delay_id, osPriorityNormal + 1);
            }
        }
}

/*
 *
 * */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
/*
 * Sensors signal - maximum measurement fault
 */
        if (htim->Instance == TIM11) {
                /* If rising edge */
                if (HAL_GPIO_ReadPin(actuator_feedback_GPIO_Port, actuator_feedback_Pin)) {
                        /* Stop machine signal */
                        stop_machine_signal_handle();

                        /* Start pulse length */
                        start_actuator_pulse = TIM11->CCR1;
                } else {
                        /* End pulse length */
                        end_actuator_pulse = TIM11->CCR1;

            			if (end_actuator_pulse > start_actuator_pulse) {
            				actuator_pulse = end_actuator_pulse - start_actuator_pulse;
            			} else {
            				actuator_pulse = (65535 - start_actuator_pulse) + end_actuator_pulse;
            			}

                        /* Stop fuse timer for XOR */
            	        osTimerStop (xor_no_signal_id);

            			/* Disable Interrupt */
                        __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_CC1);
                }
        }
}

/*
 * This callback is currently in main.c, because it is used for MxCube for freeRTOS
 *  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

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

extern TIM_HandleTypeDef htim10;
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
    if (GPIO_Pin == L_ENCODER_IND_A_Pin) {
    	data_to_master[data_to_master_3]++;
    }

    if (GPIO_Pin == L_ENCODER_IND_B_Pin) {
    	data_to_master[data_to_master_3]++;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

}

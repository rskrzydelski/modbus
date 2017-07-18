/*
 * modbus_app.h
 *
 *  Created on: 13.06.2017
 *      Author: skrzyraf
 */

#ifndef THIRD_PARTY_MODBUS_APP_LAYER_MODBUS_APP_H_
#define THIRD_PARTY_MODBUS_APP_LAYER_MODBUS_APP_H_

#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "modbus.h"

enum {
	device_status_g1, device_status_g2, data_to_master_1, data_to_master_2, data_to_master_3,
	data_to_master_4, data_to_master_5, data_to_master_6, data_to_master_7, data_to_master_8,
	DATA_TO_MASTER_NUM
};

enum {
	time_overflow_idx, data_from_master_2,
	DATA_FROM_MASTER_NUM
};

enum {
	momentary_sw_g1, momentary_sw_g2,
	SET_COILS_BY_MASTER_NUM
};

/* The volatile tells the compiler that it must read/write the variable every time the program says to do so.
 * This variables are shares between many tasks.
 *  */
volatile uint16_t data_to_master[DATA_TO_MASTER_NUM];
volatile uint16_t data_from_master[DATA_FROM_MASTER_NUM];
volatile uint8_t set_coils_by_master[SET_COILS_BY_MASTER_NUM];

/**
 * \brief Handle modbus function 0x03 (data to master)
 *
 * \param [in] uart_struct       pointer to a UART_HandleTypeDef structure that contains
 *                               the configuration information for the specified UART module.
 * \param [in] first_addr
 * \param [in] number_of_req
 *
 */
void app_data_to_master(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req);

/**
 * \brief Handle modbus function 0x10 (data from master)
 *
 * \param [in] uart_struct       pointer to a UART_HandleTypeDef structure that contains
 *                               the configuration information for the specified UART module.
 *
 */
void app_data_from_master(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req);

/**
 * \brief Handle modbus function 0x0F (set coils by master)
 *
 * This function sets bits start from first coil address
 *
 * \param [in] uart_struct           pointer to a UART_HandleTypeDef structure that contains
 *                                   the configuration information for the specified UART module.
 * \param [in]  first_coil_addr      Address of first coil
 * \param [in]  number_of_coils      Number of coil to write
 *
 */
void app_set_coils_by_master(UART_HandleTypeDef *uart_struct, uint16_t first_coil_addr, uint16_t number_of_coils);

#endif /* THIRD_PARTY_MODBUS_APP_LAYER_MODBUS_APP_H_ */

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
	data_to_master_1, data_to_master_2, data_to_master_3, data_to_master_4, data_to_master_5,
	data_to_master_6, data_to_master_7, data_to_master_8, data_to_master_9, data_to_master_10,
	DATA_TO_MASTER_NUM
};

enum {
	data_from_master_1, data_from_master_2,
	DATA_FROM_MASTER_NUM
};

uint16_t data_to_master[DATA_TO_MASTER_NUM];
uint16_t data_from_master[DATA_FROM_MASTER_NUM];

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

#endif /* THIRD_PARTY_MODBUS_APP_LAYER_MODBUS_APP_H_ */

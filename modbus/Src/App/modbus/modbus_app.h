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
	data1, data2, data3, data4, data5, data6, data7, data8, data9, data10,
	DATA_NUM
};

uint16_t data_to_master[DATA_NUM];
/**
 * \brief Handle modbus function 0x03 (data to master)
 *
 * \param [in] uart_structhuart: pointer to a UART_HandleTypeDef structure that contains
 *                               the configuration information for the specified UART module.
 *
 * \param [in] first_addr        First address
 * \param [in] number_of_req     Number of register to send to master
 *
 */
void app_data_to_master(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req);

#endif /* THIRD_PARTY_MODBUS_APP_LAYER_MODBUS_APP_H_ */

/*
 * modbus_app.c
 *
 *  Created on: 13.06.2017
 *      Author: skrzyraf
 */
#include "modbus_app.h"

void app_data_to_master(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req)
{
	        uint8_t item, i;
	        unsigned int crc;
	        uint8_t crc_l, crc_h;
	        uint8_t total_data;
	        uint8_t data_packet_length = 2 * number_of_req;

	        /* Preparing response */

	        /* Assign slave address */
	        modbus_tx_buf[0] = SLAVE_ADDRESS;

	        /* Modbus function code */
	        modbus_tx_buf[1] = DATA_TO_MASTER;
	        modbus_tx_buf[2] = data_packet_length;

	        /* Check that data are in our range */
	        if (first_addr + (data_packet_length / 2) > DATA_TO_MASTER_NUM) {
                    /* Send exception */
                    handle_exception(uart_struct, DATA_TO_MASTER, ILLEGAL_DATA_ADDRESS);
                    return;
	        }

	        /* Fill data to be send - each loop pack one word */
	        for (i = 0, item = 3; i < (data_packet_length / 2); i++, item+=2) {
	                modbus_tx_buf[item] = (data_to_master[first_addr] >> 8);
	                modbus_tx_buf[item + 1] = data_to_master[first_addr];
	                first_addr++;
	        }

	        /* Total amount of bytes to be send, 5 = slave address, function number, number of data,
	         * crc_l and crc_h */
	        total_data = data_packet_length + 5;

	        /* Crc calculation, -2 because total_data include two bytes for crc which we currently calculate */
	        crc = crc_chk(modbus_tx_buf, total_data - 2);
	        /* FIXME: Check with reialble client that endianess is correct */
	        crc_l = crc;
	        crc_h = (crc >> 8);

	        /* Add crc to buffer: 2 - slave address, function code, number of data */
	        modbus_tx_buf[2 + data_packet_length + 1] = crc_l;
	        modbus_tx_buf[2 + data_packet_length + 2] = crc_h;

	        /* Send buffer to master */
	        HAL_UART_Transmit_DMA(uart_struct, modbus_tx_buf, total_data);
}


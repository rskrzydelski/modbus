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

void app_data_from_master(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req)
{
    uint8_t item, i;
    unsigned int crc;
    uint8_t crc_l, crc_h;
    uint8_t total_data;
    uint8_t data_packet_length = 2 * number_of_req;

    /* TODO: Please check that client doesn't send 0, and it can overwrite our configuration value */

    /* Check that data are in our range */
    if (first_addr + (data_packet_length / 2) > DATA_FROM_MASTER_NUM) {
            /* Send exception */
            handle_exception(uart_struct, DATA_FROM_MASTER, ILLEGAL_DATA_ADDRESS);
            return;
    }

    /* Fill receive data to our data table - each loop pack one word */
    for (i = 0, item = 7; i < (data_packet_length / 2); i++, item+=2) {
            data_from_master[first_addr] = modbus_rx_buf[item + 1] | (modbus_rx_buf[item] << 8);
            first_addr++;
    }

    /* Preparing response */

    /* In response first 6 bytes of data are the same like query
     */
    for (i = 0; i < 6; i++) {
            modbus_tx_buf[i] = modbus_rx_buf[i];
    }

    /* Total amount of bytes to be send, 6 -> (1) slave address + (1) function number + (2) data address +
     * (2) register number + (1) crc_l + (1) crc_h */
    total_data = 8;

    /* Crc calculation, -2 because total_data include two bytes for crc which we currently calculate */
    crc = crc_chk(modbus_tx_buf, total_data - 2);
    /* FIXME: Check with reialble client that endianess is correct */
    crc_l = crc;
    crc_h = (crc >> 8);

    /* Slave address, function code, starting data address (W), content of data (W) */
    modbus_tx_buf[6] = crc_l;
    modbus_tx_buf[7] = crc_h;

    /* Send response to master */
    HAL_UART_Transmit_DMA(uart_struct, (uint8_t *)modbus_tx_buf, total_data);
}

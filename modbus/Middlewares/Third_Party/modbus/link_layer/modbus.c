/*
 * modbus.c
 *
 *  Created on: 08.06.2017
 *      Author: skrzyraf
 */
#include <stdio.h>
#include <string.h>
#include "modbus.h"

volatile request_message_info_t request_message_info;
const modbus_callbacks_t *cb = NULL;

void modbus_init(const modbus_callbacks_t *callbacks)
{
        /* Register application callbacks */
        cb = callbacks;
}

/* CRC calculation */
unsigned int crc_chk(unsigned char *data, unsigned char length)
{
        int i;
        unsigned int reg_crc = 0xFFFF;

        while (length--) {
                reg_crc ^= *data++;
                for (i = 0; i < 8; i++) {
                        if ( reg_crc & 0x01 ) { /* LSB(bit 0) = 1 */
                                reg_crc = (reg_crc >> 1)^0xA001;
                        } else {
                                reg_crc = (reg_crc >> 1);
                        }
                }
        }
        return reg_crc;
}

/* FIXME: This is not fully implementation of modbus, it's supports only few function */
void receive_modbus_message(const char data)
{
        static volatile bool modbus_enable = 0;
        /* initial length of frame on max size of buffer */
        static volatile uint8_t data_length = 11;
        /* Index of receive buffer */
        static volatile uint8_t rx_index = 0;

        /* If first byte is a slave address then start to increment index of receive buffer */
        if (data == SLAVE_ADDRESS || modbus_enable) {
                /* We catch slave address, so we suppose that data catching is started */
                modbus_enable = true;

                /* Put data to modbus receive buffer */
                modbus_rx_buf[rx_index] = data;

                /* Calculate data length based on second item (information about function)
                 * in receive buffer */
                if (rx_index == 1) {
                	switch (modbus_rx_buf[rx_index]) {
                	        case DATA_TO_MASTER:
                	                /* Data_length is needed to inform function when data catching will be finish */
                	                data_length = 8;
                	                break;
                	        case DATA_FROM_MASTER:
                	        	    /* It is set to 11 temporialy but data_length will be specify in 6 item in
                	        	     * buffer which inform about count of bytes which master want to write to us */
                                    data_length = 11;
                                    break;
                            case SET_COILS_BY_MASTER:
                                    /* It is set to 11 temporialy but data_length will be specify in 6 item in
                                     * buffer which inform about count of bytes which master want to write to us */
                                    data_length = 11;
                                    break;
                            default:
                                   /* FIXME: Please find better way to solve rubbish data */
                                   data_length = 11;
                	               break;
                	}
                }

                /* In case of that functions, length of frame is depend on number
                 * of bytes which master want write to the slave */
                if (((modbus_rx_buf[1] == DATA_FROM_MASTER) || (modbus_rx_buf[1] == SET_COILS_BY_MASTER)) && rx_index == 6) {
                        data_length = 9 + modbus_rx_buf[6];
                }

                rx_index++;

                /* If index of receive buffer is equal data length then set index on 0
                 * and data length to default value */
                if (rx_index == data_length) {
                        /* This info is needed for other function, e.g. for CRC calculation */
                        request_message_info.message_len = data_length;
                        rx_index = 0;
                        data_length = 11;
                        /* Set modbus_enable to false, because it is end of modbus data catching */
                        modbus_enable = false;
                        osSemaphoreRelease(DataReadyRxHandle);
                }
        }
}

void send_modbus_message(UART_HandleTypeDef *uart_struct)
{
        int crc;
        uint8_t request_crc_l, request_crc_h, calc_crc_l, calc_crc_h;

        /* Wait for data ready event */
        osSemaphoreWait (DataReadyRxHandle, osWaitForever);

        /* Calculate CRC */
        crc = crc_chk((unsigned char *)modbus_rx_buf, (unsigned char)request_message_info.message_len - 2);

        /* FIXME: Check with reialble client that endianess is correct */
        calc_crc_l = crc;
        calc_crc_h = (crc >> 8);

        request_crc_l = modbus_rx_buf[(request_message_info.message_len) - 2];
        request_crc_h = modbus_rx_buf[(request_message_info.message_len) - 1];

        /* Check CRC agreement */
        if (!(request_crc_l == calc_crc_l && request_crc_h == calc_crc_h)) {
        	    HAL_UART_Transmit_DMA(uart_struct, "ERR", 4);
                return;
        }

        switch(modbus_rx_buf[1]) {
                case DATA_TO_MASTER:
                        /* Pass control to application */
                        if (cb && cb->data_to_master) {
                            uint16_t first_addr = modbus_rx_buf[3] | (modbus_rx_buf[2] << 8);
				            uint16_t number_of_req = modbus_rx_buf[5] | (modbus_rx_buf[4] << 8);

                            cb->data_to_master(uart_struct, first_addr, number_of_req);
                        } else {
                            /* Send exception */
                            handle_exception(uart_struct, modbus_rx_buf[1], ILLEGAL_FUNCTION);
                        }
                        break;
                case DATA_FROM_MASTER:
                        /* Pass control to application */
                        if (cb && cb->data_from_master) {
                            uint16_t first_addr = modbus_rx_buf[3] | (modbus_rx_buf[2] << 8);
							uint16_t number_of_req = modbus_rx_buf[5] | (modbus_rx_buf[4] << 8);

                            cb->data_from_master(uart_struct, first_addr, number_of_req);
                        } else {
                            /* Send exception */
                            handle_exception(uart_struct, modbus_rx_buf[1], ILLEGAL_FUNCTION);
                        }
                        break;
                case SET_COILS_BY_MASTER:
                        /* Pass control to application */
                        if (cb && cb->set_coils_by_master) {
                            uint16_t first_coil_addr = modbus_rx_buf[3] | (modbus_rx_buf[2] << 8);
                            uint16_t number_of_coils = modbus_rx_buf[5] | (modbus_rx_buf[4] << 8);

                            cb->set_coils_by_master(uart_struct, first_coil_addr, number_of_coils);
                        } else {
                            /* Send exception */
                            handle_exception(uart_struct, modbus_rx_buf[1], ILLEGAL_FUNCTION);
                        }
                        break;
                default:
                       /* Send exception */
                       handle_exception(uart_struct, modbus_rx_buf[1], ILLEGAL_FUNCTION);
                       break;
        }
}

void handle_exception(UART_HandleTypeDef *uart_struct, uint8_t function_code, int exception_code)
{
        unsigned int crc;
        uint8_t crc_l, crc_h;

        /* Assign slave address */
        modbus_tx_buf[0] = SLAVE_ADDRESS;

        /* Set the highest bit to 1 */
        modbus_tx_buf[1] = function_code | 0x80;
        modbus_tx_buf[2] = exception_code;

        /* Crc calculation, -2 because total_data include two bytes for crc which we currently calculate */
        crc = crc_chk(modbus_tx_buf, 3);
        crc_l = crc;
        crc_h = (crc >> 8);

        /* Add crc to buffer: 2 - slave address, function code, number of data */
        modbus_tx_buf[3] = crc_l;
        modbus_tx_buf[4] = crc_h;

        /* Send buffer to master */
        HAL_UART_Transmit_DMA(uart_struct, modbus_tx_buf, 5);
}

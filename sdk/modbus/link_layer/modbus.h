/*
 * modbus.h
 *
 *  Created on: 08.06.2017
 *      Author: skrzyraf
 */

#ifndef THIRD_PARTY_MODBUS_MODBUS_H_
#define THIRD_PARTY_MODBUS_MODBUS_H_

#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Recieve buffer size */
#define UART0_RX_BUF_SIZE        128
/* Transmit buffer size */
#define UART0_TX_BUF_SIZE        128

#define SLAVE_ADDRESS                   0x01

/* Modbus functions */
#define UNKNOWN_FUNCTION                0xFF          /* In modbus spec it is called '' */
#define DATA_TO_MASTER                  0x03          /* In modbus spec it is called 'Read Holding Register' */
#define DATA_FROM_MASTER                0x10          /* In modbus spec it is called 'Write Multiple Register */
#define SET_COILS_BY_MASTER             0x0F          /* In modbus spec it is called 'Write Multiple Coils' */

/* Exception codes */
#define ILLEGAL_FUNCTION                0x01
#define ILLEGAL_DATA_ADDRESS            0x02
#define ILLEGAL_DATA_VALUE              0x03
#define SLAVE_DEVICE_FAILURE            0x04
#define SLAVE_DEVICE_BUSY               0x06

/* Transmission buffers */
volatile char modbus_rx_buf[UART0_RX_BUF_SIZE];
unsigned char modbus_tx_buf[UART0_TX_BUF_SIZE];

typedef struct {
    uint8_t message_len;
} request_message_info_t;

volatile bool modbus_timeout;
uint8_t data_in_item;

/* Event which inform that modbus data received */
osSemaphoreId DataReadyRxHandle;

/***
 * --- This part of header concern modbus application to link layer API ---
 ***/

/* Callback for handling 0x03 function by application */
typedef void (* data_to_master_cb_t)(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req);

/* Callback for handling 0x10 function by application */
typedef void (* data_from_master_cb_t)(UART_HandleTypeDef *uart_struct, uint16_t first_addr, uint16_t number_of_req);

/* Callback for handling 0x0F function by application */
typedef void (* set_coils_by_master_cb_t)(UART_HandleTypeDef *uart_struct, uint16_t first_coil_addr, uint16_t number_of_coils);

typedef struct {
	    data_to_master_cb_t data_to_master;                          /* Called when remote request 0x03 function */
	    data_from_master_cb_t data_from_master;                      /* Called when remote request 0x10 function */
	    set_coils_by_master_cb_t set_coils_by_master;                /* Called when remote request 0x0F function */
} modbus_callbacks_t;

/***
 * --- End of modbus application to link layer API ---
 ***/

/**
 * brief CRC calculation
 *
 * This function is used for crc calculation
 *
 * \param [in] data - pointer to the data which will be calculate
 * \param [in] length - data length which will be calculate
 *
 * \return crc_chk - calculated crc
 */
unsigned int crc_chk(unsigned char *data, unsigned char length);

/**
 * brief Receive data frame from buffer
 *
 * This function can be call from receive complete interrupt.
 *
 * \param [in] data - to this variable should be pass data from UDR0 buffer
 *
 */
void receive_modbus_message(const char data);

/**
 * brief Send modbus data
 *
 * This function can be call from modbus task mainloop.
 *
 * \param [in] data - to this variable should be pass data from UDR0 buffer
 *
 */
void send_modbus_message(UART_HandleTypeDef *uart_struct);

/**
 * \brief Modbus initialization
 *
 * This function assigned slave address to buffer and sets baud rate.
 *
 */
void modbus_init(const modbus_callbacks_t *callbacks);

/**
 * \brief Handle modbus function 0x03 (data to master)
 *
 * \param [in] uart_struct     Pointer to UART handle Structure
 * \param [in] data            Pointer to buffer with data for master
 * \param [in] data_len        Length of data in bytes
 *
 */
void handle_data_to_master_message(UART_HandleTypeDef *uart_struct, const uint16_t data[], int data_len);

/**
 * \brief Handle exception
 *
 * This function handled modbus exception e.g illegal function
 *
 * \param [in] function_code     modbus function code
 * \param [in] exception_code    exception code defined on top of this file
 *
 */
void handle_exception(UART_HandleTypeDef *uart_struct, uint8_t function_code, int exception_code);


#endif /* THIRD_PARTY_MODBUS_MODBUS_H_ */

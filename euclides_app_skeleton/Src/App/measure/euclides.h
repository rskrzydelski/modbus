/*
 * euclides.h
 *
 *  Created on: 23.06.2017
 *      Author: skrzyraf
 *
 *      In this file are all general device stuff like e.g statuses of device
 */

#ifndef APP_MEASURE_EUCLIDES_H_
#define APP_MEASURE_EUCLIDES_H_

#include "modbus_app.h"

/* Macros used for set and reset status device */

#define SET_STATUS_G1(a)     ((data_to_master[device_status_g1]) |= (a))
#define RST_STATUS_G1(a)     ((data_to_master[device_status_g1]) &= ~ (a))

#define SET_STATUS_G2(a)     ((data_to_master[device_status_g2]) |= (a))
#define RST_STATUS_G2(a)     ((data_to_master[device_status_g2]) &= ~ (a))

#define STATUS_G1_IS_SET(a)  ((data_to_master[device_status_1]) & (a))

#define CLEAR_ALL_ST_G1      (data_to_master[device_status_g1] = 0)
#define CLEAR_ALL_ST_G2      (data_to_master[device_status_g2] = 0)

#define RST_BUTTON_G1(a)     ((set_coils_by_master[momentary_sw_g1]) &= ~(a))

/* Device statuses - if non status is set then device is in idle state */
#define ST_G1_1                     0b0000000000000001					/* ??? */
#define ST_G1_CONFIGURATION         0b0000000000000010					/* CONFIGURATION MODE */
#define ST_G1_CURTAIN		        0b0000000000000100					/* CURTAIN MODE */
#define ST_G1_SAFETY_SWITCH         0b0000000000001000					/* SAFETY SWITCH MODE */
#define ST_G1_5				        0b0000000000010000					/*  */
#define ST_G1_6				        0b0000000000100000					/*  */
#define ST_G1_7				        0b0000000001000000					/*  */
#define ST_G1_8				        0b0000000010000000					/*  */
#define ST_G1_9				        0b0000000100000000					/*  */
#define ST_G1_10				    0b0000001000000000					/*  */
#define ST_G1_11				    0b0000010000000000					/*  */
#define ST_G1_12			        0b0000100000000000					/*  */
#define ST_G1_13				    0b0001000000000000					/*  */
#define ST_G1_14				    0b0010000000000000					/*  */
#define ST_G1_15				    0b0100000000000000					/*  */
#define ST_G1_16				    0b1000000000000000					/*  */

#define ST_G2_1                     0b0000000000000001					/*  */
#define ST_G2_2             		0b0000000000000010					/*  */
#define ST_G2_3				        0b0000000000000100					/*  */
#define ST_G2_4				        0b0000000000001000					/*  */
#define ST_G2_5				        0b0000000000010000					/*  */
#define ST_G2_6				        0b0000000000100000					/*  */
#define ST_G2_7				        0b0000000001000000					/*  */
#define ST_G2_8				        0b0000000010000000					/*  */
#define ST_G2_9				        0b0000000100000000					/*  */
#define ST_G2_10				    0b0000001000000000					/*  */
#define ST_G2_11				    0b0000010000000000					/*  */
#define ST_G2_12			        0b0000100000000000					/*  */
#define ST_G2_13				    0b0001000000000000					/*  */
#define ST_G2_14				    0b0010000000000000					/*  */
#define ST_G2_15				    0b0100000000000000					/*  */
#define ST_G2_16				    0b1000000000000000					/*  */

/* Buttons which are set by a client */
#define SW1_G1_GO_TO_CONFIG            0b00000001					/* Go to configuration task */
#define SW2_G1_GO_TO_CURTAIN           0b00000010                   /* Go to safety curtain task */
#define SW3_G1_GO_TO_SAFETY_SWITCH     0b00000100                   /* Go to safety switch task */
#define SW4_G1                         0b00001000
#define SW5_G1                         0b00010000
#define SW6_G1                         0b00100000
#define SW7_G1                         0b01000000
#define SW8_G1_TERMINATE_PROCEDURE     0b10000000

#define SW1_G2                         0b00000001					/*  */
#define SW2_G2                         0b00000010
#define SW3_G2                         0b00000100
#define SW4_G2                         0b00001000
#define SW5_G2                         0b00010000
#define SW6_G2                         0b00100000
#define SW7_G2                         0b01000000
#define SW8_G2                         0b10000000

#endif /* APP_MEASURE_EUCLIDES_H_ */

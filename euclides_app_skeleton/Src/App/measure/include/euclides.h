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
#include "config_task.h"
#include "curtain_task.h"
#include "safety_switch_task.h"

/* Callback for clean up particular procedure */
typedef void (* clean_up_cb_t)(void);

/* Global variables common for measurement procedures */
uint8_t step;

/*
 * Measurement variables
 */
volatile uint16_t machine_stop_signal_position;       /* Position remembered while stop signal */
volatile uint16_t machine_stop_velocity;              /* Velocity remebered while stop signal */
volatile uint16_t start_actuator_pulse;               /* First signal from actuator sensor */
volatile uint16_t end_actuator_pulse;                 /* Second signal from actuator sensor */
volatile uint16_t actuator_pulse;                     /* Finally actuator pulse width */

/* Variables with tail __ are a finally measurement results */
volatile uint32_t stop_machine_time__;
volatile uint16_t stop_distance__;
volatile float xor_pulse_ms__;

/* Software timers */
osTimerId xor_no_signal_id;
osTimerDef_t xor_no_signal_def;

osTimerId timer_on_delay_id;
osTimerDef_t timer_on_delay_def;

/*
 * Measurement macros
 * */
#define MACHINE_STOP_TIME          TIM2->CNT
#define MACHINE_POSITION           TIM1->CNT

/* Macros used for set and reset status device */

#define SET_STATUS_G1(a)     ((data_to_master[device_status_g1]) |= (a))
#define RST_STATUS_G1(a)     ((data_to_master[device_status_g1]) &= ~ (a))

#define SET_STATUS_G2(a)     ((data_to_master[device_status_g2]) |= (a))
#define RST_STATUS_G2(a)     ((data_to_master[device_status_g2]) &= ~ (a))

#define STATUS_G1_IS_SET(a)  ((data_to_master[device_status_g1]) & (a))

#define CLEAR_ALL_ST_G1      (data_to_master[device_status_g1] = 0)
#define CLEAR_ALL_ST_G2      (data_to_master[device_status_g2] = 0)

#define RST_BUTTON_G1(a)     ((set_coils_by_master[momentary_sw_g1]) &= ~(a))
#define RST_BUTTON_G2(a)     ((set_coils_by_master[momentary_sw_g2]) &= ~(a))

/* Device statuses - if non status is set then device is in idle state */
#define ST_G1_1                             0b0000000000000001					/* ??? */
#define ST_G1_CONFIGURATION                 0b0000000000000010					/* Configuration procedure */
#define ST_G1_CURTAIN		                0b0000000000000100					/* Curtain procedure */
#define ST_G1_SAFETY_SWITCH                 0b0000000000001000					/* Safety switch procedure */
#define ST_G1_MANUAL_MODE			        0b0000000000010000					/* Manual mode */
#define ST_G1_AUTOMATIC_MODE	            0b0000000000100000					/* Automatic mode */
#define ST_G1_IS_MOVING                     0b0000000001000000					/* Detect machine moving - enable curtain interrupt button in master app */
#define ST_G1_MEAS_IN_PROGRESS	            0b0000000010000000					/* Measurement is in progress */
#define ST_G1_MEAS_IS_DONE	                0b0000000100000000					/* Measurement is done */
#define ST_G1_XOR_FAULT_NO_SIGNAL	        0b0000001000000000					/* Something wrong with signal from actuator sensors - no signal, or too long, or only one edge */
#define ST_G1_XOR_FAULT_WRONG_DURATION      0b0000010000000000					/* Something wrong with signal from actuator sensors - signal too long or too short */
#define ST_G1_12			                0b0000100000000000					/*  */
#define ST_G1_13				            0b0001000000000000					/*  */
#define ST_G1_14				            0b0010000000000000					/*  */
#define ST_G1_15				            0b0100000000000000					/*  */
#define ST_G1_16				            0b1000000000000000					/*  */

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

#define SW1_G2_MANUAL_MODE             0b00000001					/* Set manual mode */
#define SW2_G2_AUTOMATIC_MODE          0b00000010                   /* Set automatic mode */
#define SW3_G2_POSITION_IS_ZERO        0b00000100                   /* User set machine to zero position */
#define SW4_G2_STOP_MACHINE            0b00001000                   /* Stop machine button */
#define SW5_G2                         0b00010000
#define SW6_G2                         0b00100000
#define SW7_G2                         0b01000000
#define SW8_G2                         0b10000000

#define LINEAR_ROLL_OUT              0
#define LINEAR_ROLL                  1
#define DIR							   ((TIM1->CR1 & TIM_CR1_DIR) >> 4)

/*
 * Measurement function
 * */

/**
 * \brief Stop machine signal - it sets and enable all module needed for stop machine measurement.
 * It is called after stop signal
 *
 */
void stop_machine_signal_handle(void);

/*
 * General function
 */

/**
 * \brief Terminate procedure - clean up - sets all interrupts, variables, etc after terminate procedure
 *
 */
void terminate_procedure(clean_up_cb_t clean_up_cb, osThreadId thread_id);

#endif /* APP_MEASURE_EUCLIDES_H_ */

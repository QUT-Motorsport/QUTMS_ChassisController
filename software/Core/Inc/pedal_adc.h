/*
 * pedal_adc.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#ifndef INC_PEDAL_ADC_H_
#define INC_PEDAL_ADC_H_

#include <Timer.h>
#include "window_filtering.h"

#define NUM_PEDAL_ACCEL 2
#define NUM_PEDAL_BRAKE 2

#define ACCEL_FILTER_SIZE 128

#define PEDAL_ACCEL_1_MAX
#define PEDAL_ACCEL_1_MIN
#define PEDAL_ACCEL_2_MAX
#define PEDAL_ACCEL_2_MIN
#define PEDAL_ACCEL_3_MAX
#define PEDAL_ACCEL_3_MIN

typedef struct pedal_values {
	window_filter_t pedal_accel[NUM_PEDAL_ACCEL];
	window_filter_t pedal_brake[NUM_PEDAL_BRAKE];
	window_filter_t brake_pressure;

	uint16_t raw_pedal_accel[NUM_PEDAL_ACCEL];
	uint16_t raw_pedal_accel_dma[NUM_PEDAL_ACCEL];
	uint32_t raw_pedal_brake[NUM_PEDAL_BRAKE];
	uint32_t raw_pedal_brake_dma[NUM_PEDAL_BRAKE];
	uint32_t raw_pressure_brake[1];
	uint32_t raw_pressure_brake_dma[1];
} pedal_values_t;

extern pedal_values_t current_pedal_values;

extern Timer_t timer_pedal_adc;

void setup_pedals_adc();

void pedal_adc_timer_cb(void *args);



#endif /* INC_PEDAL_ADC_H_ */

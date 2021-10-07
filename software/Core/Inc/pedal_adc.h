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
#define NUM_STEERING 2

#define ACCEL_FILTER_SIZE 32

#define PEDAL_ACCEL_0_MAX 1380
#define PEDAL_ACCEL_0_MIN 850
#define PEDAL_ACCEL_1_MAX 3220
#define PEDAL_ACCEL_1_MIN 3050

#define PEDAL_BRAKE_MIN 330
#define PEDAL_BRAKE_MAX 750

#define PEDAL_DUTY_CYCLE 1000

#define ADC_DIFF 25

#define STEER_OFFSET_0 -7.25
#define STEER_OFFSET_1 -6

#define STEER_MIN 380
#define STEER_MAX 3260

typedef struct pedal_values {
	uint16_t pedal_accel_min[NUM_PEDAL_ACCEL];
	uint16_t pedal_accel_max[NUM_PEDAL_ACCEL];

	uint16_t brake_pressure_min;
	uint16_t brake_pressure_max;

	uint16_t pedal_accel_mapped[NUM_PEDAL_ACCEL];
	uint16_t pedal_brake_mapped;

	window_filter_t pedal_accel[NUM_PEDAL_ACCEL];
	window_filter_t brake_pressure;

	uint16_t raw_pedal_accel[NUM_PEDAL_ACCEL];
	uint16_t raw_pedal_accel_dma[NUM_PEDAL_ACCEL];

	uint32_t raw_pressure_brake[1];
	uint32_t raw_pressure_brake_dma[1];

	uint16_t raw_steering[2];
	uint16_t raw_steering_dma[2];
	window_filter_t steering_angle[2];
} pedal_values_t;

extern pedal_values_t current_pedal_values;

extern ms_timer_t timer_pedal_adc;

void setup_pedals_adc();

void pedal_adc_timer_cb(void *args);

uint16_t map_value(uint16_t input, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
double map_capped(uint16_t input, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max);

extern double steering_0;
extern double steering_1;


#endif /* INC_PEDAL_ADC_H_ */

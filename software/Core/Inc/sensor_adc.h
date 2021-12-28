/*
 * sensor_adc.h
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#ifndef INC_SENSOR_ADC_H_
#define INC_SENSOR_ADC_H_

#include <stdbool.h>

#include <Timer.h>
#include <CAN_CC.h>
#include "window_filtering.h"


#define NUM_PEDAL_ACCEL 2
#define NUM_PEDAL_BRAKE 2
#define NUM_STEERING 2

#define ACCEL_FILTER_SIZE 32

#define PEDAL_ACCEL_0_MAX 2253
#define PEDAL_ACCEL_0_MIN 1460
#define PEDAL_ACCEL_1_MAX 1500
#define PEDAL_ACCEL_1_MIN 707

#define PEDAL_BRAKE_MIN 500
#define PEDAL_BRAKE_MAX 1000

#define BRAKE_MIN_ACTUATION 700

#define PEDAL_DUTY_CYCLE 1000

#define ADC_DIFF 25


#define STEER_OFFSET_0 -7.25
#define STEER_OFFSET_1 -6

#define STEER_MIN 380
#define STEER_MAX 3260

typedef struct sensor_values {
	uint16_t pedal_duty_cycle;

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

	uint16_t brake_min_actuation;

	uint16_t steering_min;
	uint16_t steering_max;

	float steering_offset[NUM_STEERING];

	uint16_t raw_steering[NUM_STEERING];
	uint16_t raw_steering_dma[NUM_STEERING];
	window_filter_t steering_angle[NUM_STEERING];

	double steering_mapped[NUM_STEERING];

	// To comply with T.4.2, specifically T.4.2.4 and T.4.2.5
	bool APPS_disable_motors;
	bool APPS_implausibility_present;
	uint32_t APPS_implausibility_start;

	// To comply with T.4.3.3
	bool BSE_disable_motors;
	bool BSE_implausibility_present;
	uint32_t BSE_implausibility_start;

	// To comply with EV.5.7
	bool pedal_disable_motors;

	bool steering_disable_TV;
	bool steering_imp_present;
	uint32_t steering_imp_start;
} sensor_values_t;

extern sensor_values_t current_sensor_values;
extern ms_timer_t timer_sensor_adc;

bool setup_adc_peripherals();
bool check_sensors_connected(CC_Flags_u *error_flags);
void setup_adc_sensors();

void sensor_adc_timer_cb(void *args);

double map_capped(uint16_t input, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max);

// todo: make these go bye bye
extern double steering_0;
extern double steering_1;


#endif /* INC_PEDAL_ADC_H_ */

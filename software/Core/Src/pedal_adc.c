/*
 * pedal_adc.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#include "main.h"
#include "pedal_adc.h"
#include "adc.h"

#include <stdio.h>
#include "CC_CAN_Messages.h"

pedal_values_t current_pedal_values;
Timer_t timer_pedal_adc;

int count = 0;

bool accel_update = false;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc1) {
		// accel
		memcpy(current_pedal_values.raw_pedal_accel,
				current_pedal_values.raw_pedal_accel_dma,
				sizeof(uint16_t) * NUM_PEDAL_ACCEL);

	} else if (hadc == &hadc3) {
		// brake pressure
		current_pedal_values.raw_pressure_brake[0] =
				current_pedal_values.raw_pressure_brake_dma[0];

	}
}
void setup_pedals_adc() {
	// every 1ms, continuous
	timer_pedal_adc = Timer_init(1, true, pedal_adc_timer_cb);

	// start dma requests
	HAL_ADC_Start_DMA(&hadc1,
			(uint32_t*) current_pedal_values.raw_pedal_accel_dma,
			NUM_PEDAL_ACCEL);
	HAL_ADC_Start_DMA(&hadc3, current_pedal_values.raw_pressure_brake_dma, 1);

	// wait for first dma round
	HAL_Delay(1);

	// setup filters
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_initialize(&current_pedal_values.pedal_accel[i],
				(uint16_t) current_pedal_values.raw_pedal_accel[i],
				ACCEL_FILTER_SIZE);
	}

	window_filter_initialize(&current_pedal_values.brake_pressure,
			current_pedal_values.raw_pressure_brake[0],
			ACCEL_FILTER_SIZE);

	// start timer
	Timer_start(&timer_pedal_adc);
}

void pedal_adc_timer_cb(void *args) {
	// update filter
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_update(&current_pedal_values.pedal_accel[i],
				current_pedal_values.raw_pedal_accel[i]);
	}

	window_filter_update(&current_pedal_values.brake_pressure,
			current_pedal_values.raw_pressure_brake[0]);

	count++;

	if (count == 10) {
		count = 0;

		// print raw values
		printf(
				"%i %i\t%i\r\n", ///*\t%i %i\t %i\r\n",
				current_pedal_values.pedal_accel[0].current_filtered,
				current_pedal_values.pedal_accel[1].current_filtered,
				current_pedal_values.brake_pressure.current_filtered);

		// log to CAN
		CC_TransmitPedals_t msg = Compose_CC_TransmitPedals(
				current_pedal_values.pedal_accel[0].current_filtered,
				current_pedal_values.pedal_accel[1].current_filtered,
				current_pedal_values.brake_pressure.current_filtered);
	}

}

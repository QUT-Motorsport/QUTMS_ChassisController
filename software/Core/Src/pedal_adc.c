/*
 * pedal_adc.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#include "main.h"
#include "pedal_adc.h"
#include "adc.h"
#include <math.h>

#include <stdio.h>
#include "CC_CAN_Messages.h"

pedal_values_t current_pedal_values;
ms_timer_t timer_pedal_adc;

int count = 0;

bool accel_update = false;

double steering_0 = 0;
double steering_1 = 0;

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
	} else if (hadc == &hadc2) {
		current_pedal_values.raw_steering[0] =
				current_pedal_values.raw_steering_dma[0];
		current_pedal_values.raw_steering[1] =
				current_pedal_values.raw_steering_dma[1];
	}
}
void setup_pedals_adc() {
	// every 1ms, continuous
	timer_pedal_adc = timer_init(2, true, pedal_adc_timer_cb);

	// start dma requests
	HAL_ADC_Start_DMA(&hadc1,
			(uint32_t*) current_pedal_values.raw_pedal_accel_dma,
			NUM_PEDAL_ACCEL);
	HAL_ADC_Start_DMA(&hadc3, current_pedal_values.raw_pressure_brake_dma, 1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) current_pedal_values.raw_steering_dma,
	NUM_STEERING);

	// wait for first dma round
	HAL_Delay(1);

	// setup filters
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_initialize(&current_pedal_values.pedal_accel[i],
				(uint16_t) current_pedal_values.raw_pedal_accel[i],
				ACCEL_FILTER_SIZE);
	}

	current_pedal_values.pedal_accel_min[0] = PEDAL_ACCEL_0_MIN;
	current_pedal_values.pedal_accel_max[0] = PEDAL_ACCEL_0_MAX;
	current_pedal_values.pedal_accel_min[1] = PEDAL_ACCEL_1_MIN;
	current_pedal_values.pedal_accel_max[1] = PEDAL_ACCEL_1_MAX;

	current_pedal_values.brake_pressure_min = PEDAL_BRAKE_MIN;
	current_pedal_values.brake_pressure_max = PEDAL_BRAKE_MAX;

	window_filter_initialize(&current_pedal_values.brake_pressure,
			current_pedal_values.raw_pressure_brake[0],
			ACCEL_FILTER_SIZE);

	for (int i = 0; i < NUM_STEERING; i++) {
		window_filter_initialize(&current_pedal_values.steering_angle[i],
				current_pedal_values.raw_steering[i],
				ACCEL_FILTER_SIZE);
	}

	// start timer
	timer_start(&timer_pedal_adc);
}

void pedal_adc_timer_cb(void *args) {
	// update filter
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_update(&current_pedal_values.pedal_accel[i],
				current_pedal_values.raw_pedal_accel[i]);

		uint16_t pedal_value =
				current_pedal_values.pedal_accel[i].current_filtered;

		// update min/max?
		if (pedal_value < current_pedal_values.pedal_accel_min[i]) {
			/*if (pedal_value > current_pedal_values.pedal_accel_min[i] - ADC_DIFF) {
			 printf("update min %i from %i to %i\r\n", i,
			 current_pedal_values.pedal_accel_min[i], pedal_value);
			 current_pedal_values.pedal_accel_min[i] = pedal_value;
			 } else {*/
			pedal_value = current_pedal_values.pedal_accel_min[i];
			//}
		}

		if (pedal_value > current_pedal_values.pedal_accel_max[i]) {
			/*if (pedal_value < current_pedal_values.pedal_accel_max[i] + ADC_DIFF) {
			 printf("update max %i from %i to %i\r\n", i,
			 current_pedal_values.pedal_accel_max[i], pedal_value);
			 current_pedal_values.pedal_accel_max[i] = pedal_value;
			 } else {*/
			pedal_value = current_pedal_values.pedal_accel_max[i];
			//}
		}

		current_pedal_values.pedal_accel_mapped[i] = map_value(pedal_value,
						current_pedal_values.pedal_accel_min[i],
						current_pedal_values.pedal_accel_max[i], 0,
						PEDAL_DUTY_CYCLE);
	}

	// correct accel 1 orientation
	current_pedal_values.pedal_accel_mapped[1] = PEDAL_DUTY_CYCLE - current_pedal_values.pedal_accel_mapped[1];

	window_filter_update(&current_pedal_values.brake_pressure,
			current_pedal_values.raw_pressure_brake[0]);

	for (int i = 0; i < NUM_STEERING; i++) {
		window_filter_update(&current_pedal_values.steering_angle[i],
				current_pedal_values.raw_steering[i]);
	}

	uint16_t brake_val = current_pedal_values.brake_pressure.current_filtered;

	if (brake_val > current_pedal_values.brake_pressure_max) {
		brake_val = current_pedal_values.brake_pressure_max;
	}

	if (brake_val < current_pedal_values.brake_pressure_min) {
		brake_val = current_pedal_values.brake_pressure_min;
	}

	// update min/max?

	current_pedal_values.pedal_brake_mapped = map_value(
			brake_val,
			current_pedal_values.brake_pressure_min,
			current_pedal_values.brake_pressure_max, 0,
			PEDAL_DUTY_CYCLE);

	// safety check for disconnect
	if (current_pedal_values.pedal_accel[0].current_filtered < 50) {
		current_pedal_values.pedal_accel_mapped[0] = 0;
	}

	// map to 0-1000

	count++;

	if (count >= 50) {
		count = 0;

		// log to CAN
		CC_TransmitPedals_t msg = Compose_CC_TransmitPedals(
				current_pedal_values.pedal_accel_mapped[0],
				current_pedal_values.pedal_accel_mapped[1],
				current_pedal_values.pedal_brake_mapped,
				current_pedal_values.brake_pressure.current_filtered);

		CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
		CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
				.TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &header, msg.data);
		/*
		 // convert steering angle to radians
		 double steering_0 = ((STEER_MAX
		 - current_pedal_values.steering_angle[0].current_filtered) * 360
		 / STEER_MAX);
		 double steering_1 =
		 ((current_pedal_values.steering_angle[1].current_filtered) * 360
		 / STEER_MAX);
		 */

		steering_0 = map_capped(
				current_pedal_values.steering_angle[0].current_filtered,
				STEER_MIN, STEER_MAX, 0, 360);
		steering_1 = 360
				- map_capped(
						current_pedal_values.steering_angle[1].current_filtered,
						STEER_MIN, STEER_MAX, 0, 360);

		// flip angle
		steering_0 = 180 - steering_0;
		steering_1 = 180 - steering_1;

		steering_0 += STEER_OFFSET_0;
		steering_1 += STEER_OFFSET_1;

		CC_TransmitSteering_t msg2 = Compose_CC_TransmitSteering(
				(uint16_t) (steering_0 + 180), (uint16_t) (steering_1 + 180));
		/*
		 CC_TransmitSteering_t msg2 = Compose_CC_TransmitSteering(
		 current_pedal_values.steering_angle[0].current_filtered,
		 current_pedal_values.steering_angle[1].current_filtered);
		 */
		header.ExtId = msg2.id;
		header.DLC = sizeof(msg2.data);
		CC_send_can_msg(&hcan2, &header, msg2.data);

#if PRINT_RAW_PEDALS == 1

		float pa0 = (-15 * (22+15)/(22.0f) * current_pedal_values.pedal_accel[0].current_filtered / 1000.0f) + 37.5;
		float pa1 = (-15 * (22+15)/(22.0f) * current_pedal_values.pedal_accel[1].current_filtered / 1000.0f) + 37.5;

		uint8_t apps = 0;

				int diff = abs(current_pedal_values.pedal_accel_mapped[0] - current_pedal_values.pedal_accel_mapped[1]);

				apps = diff > 100 ? 1 : 0;

		 printf("%i\t%i\t%i\t%i\t%i\t%i\t%i %i\r\n", current_pedal_values.pedal_accel[0].current_filtered, (int)pa0,current_pedal_values.pedal_accel_mapped[0],
				current_pedal_values.pedal_accel[1].current_filtered, (int)pa1, current_pedal_values.pedal_accel_mapped[1],
				apps, diff);

		/*
		 printf("%i %i\r\n", current_pedal_values.pedal_accel[0].current_filtered,
				current_pedal_values.pedal_accel[1].current_filtered);
				*/
/*
		uint8_t apps = 0;

		int diff = abs(current_pedal_values.pedal_accel_mapped[0] - current_pedal_values.pedal_accel_mapped[1]);

		apps = diff > 100 ? 1 : 0;

		printf("%i %i %i %i\r\n", current_pedal_values.pedal_accel_mapped[0],
				current_pedal_values.pedal_accel_mapped[1], apps, diff);
*/
		/*printf("%i %i %i %i\r\n",
		 current_pedal_values.brake_pressure.current_filtered,
		 current_pedal_values.brake_pressure_min,
		 current_pedal_values.brake_pressure_max,
		 current_pedal_values.pedal_brake_mapped);
		 */
#endif
	}
}

uint16_t map_value(uint16_t input, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max) {
	return (input - in_min) * (out_max - out_min) / (float) (in_max - in_min)
			+ out_min;
}

double map_capped(uint16_t input, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max) {
	if (input < in_min) {
		input = in_min;
	} else if (input > in_max) {
		input = in_max;
	}

	return (double) (input - in_min) * (double) (out_max - out_min)
			/ (double) (in_max - in_min) + (double) out_min;
}

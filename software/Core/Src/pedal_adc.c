/*
 * pedal_adc.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#include <math.h>
#include <stdio.h>

#include <CC_CAN_Messages.h>

#include "main.h"
#include "pedal_adc.h"
#include "adc.h"
#include "debugCAN.h"

pedal_values_t current_pedal_values;
ms_timer_t timer_pedal_adc;

int count = 0;

bool accel_update = false;

double steering_0 = 0;
double steering_1 = 0;

void update_pedal_values();
void update_APPS();
void update_BSE();
void update_pedal_plausibility();

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

void update_pedal_values() {
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
	current_pedal_values.pedal_accel_mapped[1] = PEDAL_DUTY_CYCLE
			- current_pedal_values.pedal_accel_mapped[1];

	// update brake pressure
	window_filter_update(&current_pedal_values.brake_pressure,
			current_pedal_values.raw_pressure_brake[0]);

	uint16_t brake_val = current_pedal_values.brake_pressure.current_filtered;

	if (brake_val > current_pedal_values.brake_pressure_max) {
		brake_val = current_pedal_values.brake_pressure_max;
	}

	if (brake_val < current_pedal_values.brake_pressure_min) {
		brake_val = current_pedal_values.brake_pressure_min;
	}

	current_pedal_values.pedal_brake_mapped = map_value(brake_val,
			current_pedal_values.brake_pressure_min,
			current_pedal_values.brake_pressure_max, 0,
			PEDAL_DUTY_CYCLE);

	// update steering
	for (int i = 0; i < NUM_STEERING; i++) {
		window_filter_update(&current_pedal_values.steering_angle[i],
				current_pedal_values.raw_steering[i]);
	}

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
}

void update_APPS() {
	bool APPS_implausibility_check = false;

	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		if (current_pedal_values.pedal_accel[i].current_filtered
				< ADC_DISCONNECT_CUTOFF) {
			APPS_implausibility_check = true;
		}
	}

	int diff = abs(
			current_pedal_values.pedal_accel_mapped[0]
					- current_pedal_values.pedal_accel_mapped[1]);
	if (diff > APPS_DIFF) {
		APPS_implausibility_check = true;
	}

	current_pedal_values.APPS_implausibility_present =
			APPS_implausibility_check;

	if (current_pedal_values.APPS_implausibility_present) {
		// current implausibility detected
		if (current_pedal_values.APPS_implausibility_start == 0) {
			current_pedal_values.APPS_implausibility_start = HAL_GetTick();
		}

		if ((HAL_GetTick() - current_pedal_values.APPS_implausibility_start)
				> PEDAL_IMPLAUSIBILITY_TIMEOUT) {
			// 100ms of implausibility, so ensure we can't send motor values
			current_pedal_values.APPS_disable_motors = true;
		}
	} else {
		current_pedal_values.APPS_implausibility_start = 0;
		current_pedal_values.APPS_disable_motors = false;
	}
}

void update_BSE() {
	// TODO: calculate correct status based off brake pressure sensor
	current_pedal_values.BSE_implausibility_present = false;

	if (current_pedal_values.brake_pressure.current_filtered
			< ADC_DISCONNECT_CUTOFF) {
		current_pedal_values.BSE_implausibility_present = true;
	}

	if (current_pedal_values.BSE_implausibility_present) {
		// current implausibility detected
		if (current_pedal_values.BSE_implausibility_start == 0) {
			current_pedal_values.BSE_implausibility_start = HAL_GetTick();
		}

		if ((HAL_GetTick() - current_pedal_values.BSE_implausibility_start)
				> PEDAL_IMPLAUSIBILITY_TIMEOUT) {
			// 100ms of implausibility, so ensure we can't send motor values
			current_pedal_values.BSE_disable_motors = true;
		}
	} else {
		current_pedal_values.BSE_implausibility_start = 0;
		current_pedal_values.BSE_disable_motors = false;
	}
}

void update_pedal_plausibility() {
	// TODO: check via accel pedal and brake pressure

	// if brake AND accel > 25% -> pedal_disable_motors = true
	if ((current_pedal_values.pedal_brake_mapped > BRAKE_MIN_ACTIVATION) && (current_pedal_values.pedal_accel_mapped[0] > 250)) {
		current_pedal_values.pedal_disable_motors = true;
	}


	// if accel < 5% -> pedal_disable_motors = false
	if (current_pedal_values.pedal_disable_motors) {
		if (current_pedal_values.pedal_accel_mapped[0] < 50) {
			current_pedal_values.pedal_disable_motors = false;
		}
	}
}

void pedal_adc_timer_cb(void *args) {

	update_pedal_values();

	// CHECK PEDAL FAULTS

	update_APPS();
	update_BSE();
	update_pedal_plausibility();

	// TODO: remove, this is covered by plausibility

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

		// send any error status
		if (current_pedal_values.APPS_disable_motors) {
			// send APPS error
			debugCAN_errorPresent(DEBUG_ERROR_APPS_IMPLAUSIBILITY);
		}

		if (current_pedal_values.BSE_disable_motors) {
			// send BSE error
			debugCAN_errorPresent(DEBUG_ERROR_BSE_IMPLAUSIBILITY);
		}

		if (current_pedal_values.pedal_disable_motors) {
			// send pedal plausibility error
			debugCAN_errorPresent(DEBUG_ERROR_PEDAL_IMPLAUSIBILITY);
		}

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
		/*
		 float pa0 =
		 (-15 * (22 + 15) / (22.0f)
		 * current_pedal_values.pedal_accel[0].current_filtered
		 / 1000.0f) + 37.5;
		 float pa1 =
		 (-15 * (22 + 15) / (22.0f)
		 * current_pedal_values.pedal_accel[1].current_filtered
		 / 1000.0f) + 37.5;

		 uint8_t apps = 0;

		 int diff = abs(
		 current_pedal_values.pedal_accel_mapped[0]
		 - current_pedal_values.pedal_accel_mapped[1]);

		 apps = diff > APPS_DIFF ? 1 : 0;

		 printf("%li\t%i\t%i\t%li\t%i\t%i\t%i %i %i\r\n",
		 current_pedal_values.pedal_accel[0].current_filtered, (int) pa0,
		 current_pedal_values.pedal_accel_mapped[0],
		 current_pedal_values.pedal_accel[1].current_filtered, (int) pa1,
		 current_pedal_values.pedal_accel_mapped[1], apps, current_pedal_values.APPS_disable_motors ? 1 : 0, diff);
		 */
/*
		printf("a: %i, b: %i, p: %i \t a: %i \t b: %i\r\n",
				current_pedal_values.APPS_disable_motors ? 1 : 0,
				current_pedal_values.BSE_disable_motors ? 1 : 0,
				current_pedal_values.pedal_disable_motors ? 1 : 0,
						current_pedal_values.pedal_accel_mapped[0],
						current_pedal_values.pedal_brake_mapped);
*/
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
		/*
		printf("%i %i %i %i\r\n",
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

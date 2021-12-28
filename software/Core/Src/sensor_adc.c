/*
 * pedal_adc.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#include <CAN_CC.h>
#include <math.h>
#include <stdio.h>

#include "main.h"
#include "can.h"
#include "sensor_adc.h"
#include "adc.h"
#include "debugCAN.h"
#include "heartbeat.h"

sensor_values_t current_sensor_values;
ms_timer_t timer_sensor_adc;

int sensor_transmit_count = 0;

bool accel_update = false;

// todo: make these go bye bye
double steering_0 = 0;
double steering_1 = 0;

void update_sensor_values();
void update_APPS();
void update_BSE();
void update_pedal_plausibility();
void update_steering_plausibility();

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc1) {
		// accel
		memcpy(current_sensor_values.raw_pedal_accel, current_sensor_values.raw_pedal_accel_dma,
				sizeof(uint16_t) * NUM_PEDAL_ACCEL);

	}
	else if (hadc == &hadc3) {
		// brake pressure
		current_sensor_values.raw_pressure_brake[0] = current_sensor_values.raw_pressure_brake_dma[0];
	}
	else if (hadc == &hadc2) {
		current_sensor_values.raw_steering[0] = current_sensor_values.raw_steering_dma[0];
		current_sensor_values.raw_steering[1] = current_sensor_values.raw_steering_dma[1];
	}
}

bool setup_adc_peripherals() {
	// start dma requests

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) current_sensor_values.raw_pedal_accel_dma,
	NUM_PEDAL_ACCEL) != HAL_OK) {
		return false;
	}

	if (HAL_ADC_Start_DMA(&hadc3, current_sensor_values.raw_pressure_brake_dma, 1) != HAL_OK) {
		return false;
	}

	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*) current_sensor_values.raw_steering_dma,
	NUM_STEERING) != HAL_OK) {
		return false;
	}

	// small delay to ensure the first round can complete
	HAL_Delay(1);

	return true;
}

bool check_sensors_connected(CC_Flags_u *error_flags) {
	bool success = true;

	if (current_sensor_values.raw_pedal_accel[0] < ADC_DISCONNECT_CUTOFF) {
		error_flags->S_Accel0 = 1;

		success = false;
	}
	else {
		error_flags->S_Accel0 = 0;
	}

	if (current_sensor_values.raw_pedal_accel[1] < ADC_DISCONNECT_CUTOFF) {
		error_flags->S_Accel1 = 1;

		success = false;
	}
	else {
		error_flags->S_Accel1 = 0;
	}

	if (current_sensor_values.raw_pressure_brake[0] < ADC_DISCONNECT_CUTOFF) {
		error_flags->S_Brake = 1;

		success = false;
	}
	else {
		error_flags->S_Brake = 0;
	}

	if (current_sensor_values.raw_steering[0] < ADC_DISCONNECT_CUTOFF) {
		error_flags->S_Steering0 = 1;

#if STEERING_NON_CRITICAL == 0
		success = false;
#endif
	}
	else {
		error_flags->S_Steering0 = 0;
	}

	if (current_sensor_values.raw_steering[1] < ADC_DISCONNECT_CUTOFF) {
		error_flags->S_Steering1 = 1;

#if STEERING_NON_CRITICAL == 0
		success = false;
#endif
	}
	else {
		error_flags->S_Steering1 = 0;
	}

	return success;
}

void setup_adc_sensors() {
	// every 2ms, continuous
	timer_sensor_adc = timer_init(2, true, sensor_adc_timer_cb);

	// setup filters
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_initialize(&current_sensor_values.pedal_accel[i],
				(uint16_t) current_sensor_values.raw_pedal_accel[i],
				ACCEL_FILTER_SIZE);
	}

	window_filter_initialize(&current_sensor_values.brake_pressure, current_sensor_values.raw_pressure_brake[0],
	ACCEL_FILTER_SIZE);

	for (int i = 0; i < NUM_STEERING; i++) {
		window_filter_initialize(&current_sensor_values.steering_angle[i], current_sensor_values.raw_steering[i],
		ACCEL_FILTER_SIZE);
	}

	// setup constants
	current_sensor_values.pedal_duty_cycle = PEDAL_DUTY_CYCLE;

	current_sensor_values.pedal_accel_min[0] = PEDAL_ACCEL_0_MIN;
	current_sensor_values.pedal_accel_max[0] = PEDAL_ACCEL_0_MAX;
	current_sensor_values.pedal_accel_min[1] = PEDAL_ACCEL_1_MIN;
	current_sensor_values.pedal_accel_max[1] = PEDAL_ACCEL_1_MAX;

	current_sensor_values.brake_pressure_min = PEDAL_BRAKE_MIN;
	current_sensor_values.brake_pressure_max = PEDAL_BRAKE_MAX;

	current_sensor_values.steering_min = STEER_MIN;
	current_sensor_values.steering_max = STEER_MAX;

	current_sensor_values.steering_offset[0] = STEER_OFFSET_0;
	current_sensor_values.steering_offset[1] = STEER_OFFSET_1;

	// init count
	sensor_transmit_count = 0;

	// set all plausibility checks off to start

	current_sensor_values.APPS_disable_motors = false;
	current_sensor_values.BSE_disable_motors = false;
	current_sensor_values.pedal_disable_motors = false;

	// start timer
	timer_start(&timer_sensor_adc);
}

void update_sensor_values() {
	// update filter
	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		window_filter_update(&current_sensor_values.pedal_accel[i], current_sensor_values.raw_pedal_accel[i]);

		current_sensor_values.pedal_accel_mapped[i] = map_capped(current_sensor_values.pedal_accel[i].current_filtered,
				current_sensor_values.pedal_accel_min[i], current_sensor_values.pedal_accel_max[i], 0,
				current_sensor_values.pedal_duty_cycle);
	}

	// correct accel 1 orientation
	current_sensor_values.pedal_accel_mapped[1] = current_sensor_values.pedal_duty_cycle
			- current_sensor_values.pedal_accel_mapped[1];

	// update brake pressure
	window_filter_update(&current_sensor_values.brake_pressure, current_sensor_values.raw_pressure_brake[0]);

	current_sensor_values.pedal_brake_mapped = (uint16_t) map_capped(
			current_sensor_values.brake_pressure.current_filtered, current_sensor_values.brake_pressure_min,
			current_sensor_values.brake_pressure_max, 0, current_sensor_values.pedal_duty_cycle);

	// update steering angle
	for (int i = 0; i < NUM_STEERING; i++) {
		window_filter_update(&current_sensor_values.steering_angle[i], current_sensor_values.raw_steering[i]);

		current_sensor_values.steering_mapped[i] = map_capped(current_sensor_values.steering_angle[i].current_filtered,
				current_sensor_values.steering_min, current_sensor_values.steering_max, 0, 360);

		current_sensor_values.steering_mapped[i] = 180 - current_sensor_values.steering_mapped[i];

	}

	// correct steering 1 orientation
	current_sensor_values.steering_mapped[1] = -current_sensor_values.steering_mapped[1];

	// zero steering angle values
	current_sensor_values.steering_mapped[0] += current_sensor_values.steering_offset[0];
	current_sensor_values.steering_mapped[1] += current_sensor_values.steering_offset[1];
}

void update_APPS() {
	bool APPS_implausibility_check = false;

	for (int i = 0; i < NUM_PEDAL_ACCEL; i++) {
		if (current_sensor_values.pedal_accel[i].current_filtered < ADC_DISCONNECT_CUTOFF) {
			APPS_implausibility_check = true;
		}
	}

	int diff = abs(current_sensor_values.pedal_accel_mapped[0] - current_sensor_values.pedal_accel_mapped[1]);
	if (diff > APPS_DIFF) {
		APPS_implausibility_check = true;
	}

	current_sensor_values.APPS_implausibility_present = APPS_implausibility_check;

	if (current_sensor_values.APPS_implausibility_present) {
		// current implausibility detected
		if (current_sensor_values.APPS_implausibility_start == 0) {
			current_sensor_values.APPS_implausibility_start = HAL_GetTick();
		}

		if ((HAL_GetTick() - current_sensor_values.APPS_implausibility_start) > SENSOR_IMPLAUSIBILITY_TIMEOUT) {
			// 100ms of implausibility, so ensure we can't send motor values
			current_sensor_values.APPS_disable_motors = true;

			// set heartbeat flag
			CC_heartbeatState.errorFlags.IMP_APPS = 1;
		}
	}
	else {
		current_sensor_values.APPS_implausibility_start = 0;
		current_sensor_values.APPS_disable_motors = false;

		// clear heartbeat flag
		CC_heartbeatState.errorFlags.IMP_APPS = 0;
	}
}

void update_BSE() {
	current_sensor_values.BSE_implausibility_present = false;

	if (current_sensor_values.brake_pressure.current_filtered < ADC_DISCONNECT_CUTOFF) {
		current_sensor_values.BSE_implausibility_present = true;
	}

	if (current_sensor_values.BSE_implausibility_present) {
		// current implausibility detected
		if (current_sensor_values.BSE_implausibility_start == 0) {
			current_sensor_values.BSE_implausibility_start = HAL_GetTick();
		}

		if ((HAL_GetTick() - current_sensor_values.BSE_implausibility_start) > SENSOR_IMPLAUSIBILITY_TIMEOUT) {
			// 100ms of implausibility, so ensure we can't send motor values
			current_sensor_values.BSE_disable_motors = true;

			// set heartbeat flag
			CC_heartbeatState.errorFlags.IMP_BSE = 1;
		}
	}
	else {
		current_sensor_values.BSE_implausibility_start = 0;
		current_sensor_values.BSE_disable_motors = false;

		// clear heartbeat flag
		CC_heartbeatState.errorFlags.IMP_BSE = 0;
	}
}

void update_pedal_plausibility() {
	// if brake AND accel > 25% -> pedal_disable_motors = true
	if ((current_sensor_values.pedal_brake_mapped > BRAKE_MIN_ACTIVATION)
			&& (current_sensor_values.pedal_accel_mapped[0] > 250)) {
		current_sensor_values.pedal_disable_motors = true;

		// set heartbeat flag
		CC_heartbeatState.errorFlags.IMP_Pedal = 1;
	}

	// if accel < 5% -> pedal_disable_motors = false
	if (current_sensor_values.pedal_disable_motors) {
		if ((current_sensor_values.pedal_accel_mapped[0] < 50) || (current_sensor_values.pedal_accel_mapped[1] < 50)) {
			current_sensor_values.pedal_disable_motors = false;

			// clear heartbeat flag
			CC_heartbeatState.errorFlags.IMP_Pedal = 0;
		}
	}

}

void update_steering_plausibility() {
	current_sensor_values.steering_imp_present = false;

	for (int i = 0; i < NUM_STEERING; i++) {
		if (current_sensor_values.steering_angle[i].current_filtered < ADC_DISCONNECT_CUTOFF) {
			current_sensor_values.steering_imp_present = true;
		}
	}

	double diff = fabs(current_sensor_values.steering_mapped[0] - current_sensor_values.steering_mapped[1]);
	if (diff > STEER_IMP_DIFF) {
		current_sensor_values.steering_imp_present = true;
	}

	if (current_sensor_values.steering_imp_present) {
		// current implausibility detected
		if (current_sensor_values.steering_imp_start == 0) {
			current_sensor_values.steering_imp_start = HAL_GetTick();
		}

		if ((HAL_GetTick() - current_sensor_values.steering_imp_start) > SENSOR_IMPLAUSIBILITY_TIMEOUT) {
			// 100ms of implausibility, so disable TV
			current_sensor_values.steering_disable_TV = true;

			// set heartbeat flag
			CC_heartbeatState.errorFlags.IMP_Steering = 1;
		}
	}
	else {
		current_sensor_values.steering_imp_start = 0;

		// dont want TV to just snap back on, so don't reenable TV
		// current_sensor_values.steering_disable_TV = false;

		// clear heartbeat flag
		CC_heartbeatState.errorFlags.IMP_Steering = 0;
	}
}

void sensor_adc_timer_cb(void *args) {

	update_sensor_values();

	// CHECK PEDAL FAULTS
	update_APPS();
	update_BSE();
	update_pedal_plausibility();
	update_steering_plausibility();

	// this timer runs every 2ms, want to send updated pedal values every 100ms, so only send every 50 calls
	sensor_transmit_count++;

	if (sensor_transmit_count >= 50) {
		sensor_transmit_count = 0;

		// log to CAN
		CC_TransmitPedals_t msg = Compose_CC_TransmitPedals(current_sensor_values.pedal_accel_mapped[0],
				current_sensor_values.pedal_accel_mapped[1], current_sensor_values.pedal_brake_mapped,
				current_sensor_values.brake_pressure.current_filtered);

		CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
		CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &header, msg.data);

		// send any error status
		if (current_sensor_values.APPS_disable_motors) {
			// send APPS error
			debugCAN_errorPresent(DEBUG_ERROR_APPS_IMPLAUSIBILITY);
		}

		if (current_sensor_values.BSE_disable_motors) {
			// send BSE error
			debugCAN_errorPresent(DEBUG_ERROR_BSE_IMPLAUSIBILITY);
		}

		if (current_sensor_values.pedal_disable_motors) {
			// send pedal plausibility error
			debugCAN_errorPresent(DEBUG_ERROR_PEDAL_IMPLAUSIBILITY);
		}

		CC_TransmitSteering_t msg2 = Compose_CC_TransmitSteering(
				(int16_t) (current_sensor_values.steering_mapped[0] * 10),
				(int16_t) (current_sensor_values.steering_mapped[1] * 10));

		header.ExtId = msg2.id;
		header.DLC = sizeof(msg2.data);
		CC_send_can_msg(&hcan2, &header, msg2.data);

#if PRINT_RAW_PEDALS == 1
		printf("%i %i %i %i\r\n",
		 current_sensor_values.brake_pressure.current_filtered,
		 current_sensor_values.brake_pressure_min,
		 current_sensor_values.brake_pressure_max,
		 current_sensor_values.pedal_brake_mapped);

#endif
	}
}

double map_capped(uint16_t input, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	if (input < in_min) {
		input = in_min;
	}
	else if (input > in_max) {
		input = in_max;
	}

	return (double) (input - in_min) * (double) (out_max - out_min) / (double) (in_max - in_min) + (double) out_min;
}

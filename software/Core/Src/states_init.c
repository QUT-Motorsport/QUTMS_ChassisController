/*
 * CC_states_init.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"
#include "can_dict.h"
#include "heartbeat.h"
#include "sensor_adc.h"

state_t state_start = { &state_start_enter, &state_start_body, CC_STATE_START };
state_t state_pInit = { &state_pInit_enter, &state_pInit_body, CC_STATE_PERIPHERAL_INIT };
state_t state_sInit = { &state_sInit_enter, &state_sInit_body, CC_STATE_SENSOR_INIT };
state_t state_boardCheck = { &state_boardCheck_enter, &state_boardCheck_body, CC_STATE_BOARD_CHECK };
state_t state_checkAMS = { &state_checkAMS_enter, &state_checkAMS_body, CC_STATE_AMS_CHECK };
state_t state_error = { &state_error_enter, &state_error_body, CC_STATE_ERROR };

uint32_t peripheral_retry_start = 0;
uint32_t peripheral_timeout_start = 0;

uint32_t sensor_retry_start = 0;
uint32_t sensor_timeout_start = 0;

void state_start_enter(fsm_t *fsm) {
	// init object dictionary
	CC_OD_init();

	CC_heartbeatState.errorFlags.rawMem = 0;

	// go to peripheral init
	fsm_changeState(fsm, &state_pInit, "Init Peripherals");
}

void state_start_body(fsm_t *fsm) {
	return;
}

void state_pInit_enter(fsm_t *fsm) {
	// setup CAN
	bool success = true;

	if (!setup_CAN()) {
		CC_heartbeatState.errorFlags.P_CAN = 1;
		success = false;
	}

	if (!setup_adc_peripherals()) {
		CC_heartbeatState.errorFlags.P_ADC = 1;
		success = false;
	}

	if (success) {
		fsm_changeState(fsm, &state_sInit, "Peripherals initialized");
		peripheral_retry_start = 0;
		peripheral_timeout_start = 0;
	}
	else {
		// something failed, so start timers so we can retry
		peripheral_retry_start = HAL_GetTick();
		peripheral_timeout_start = HAL_GetTick();
	}
}

void state_pInit_body(fsm_t *fsm) {
	if (peripheral_timeout_start == 0) {
		// everything is initialized so skip this iteration
		return;
	}

	// if we're here, something didn't initialize
	if ((peripheral_retry_start - HAL_GetTick()) > PERIPHERAL_RETRY) {
		uint8_t success = 0;
		if (CC_heartbeatState.errorFlags.P_CAN == 1) {
			success |= (1 << 0);

			// retry CAN
			if (setup_CAN()) {
				success &= ~(1 << 0);
				CC_heartbeatState.errorFlags.P_CAN = 0;
			}
		}

		if (CC_heartbeatState.errorFlags.P_ADC == 1) {
			success |= (1 << 1);

			// retry ADC
			if (setup_adc_peripherals()) {
				success &= ~(1 << 1);
				CC_heartbeatState.errorFlags.P_ADC = 0;
			}
		}

		if (success == 0) {
			// everything has initialized correctly

			// disable timeout
			peripheral_timeout_start = 0;

			fsm_changeState(fsm, &state_sInit, "Peripherals initialized");
			return;
		}
		else {
			// something failed, so lets retry again in 100ms
			peripheral_retry_start = HAL_GetTick();
		}
	}

	if ((peripheral_timeout_start - HAL_GetTick()) > PERIPHERAL_TIMEOUT) {
		// something is clearly broken and hasn't been fixed so go to error state
		fsm_changeState(fsm, &state_error, "Peripherals failed");
		return;
	}
}

void state_sInit_enter(fsm_t *fsm) {
	if (!check_sensors_connected(&(CC_heartbeatState.errorFlags))) {
		// a sensor failed, so start timers so we can retry
		sensor_retry_start = HAL_GetTick();
		sensor_timeout_start = HAL_GetTick();
	}
	else {
		// sensors good, so setup filters and move to board check
		setup_adc_sensors();

		fsm_changeState(fsm, &state_boardCheck, "Sensors initialized");

		sensor_retry_start = 0;
		sensor_timeout_start = 0;
		return;
	}
}

void state_sInit_body(fsm_t *fsm) {
	if (sensor_timeout_start == 0) {
		// everything is connected so skip this iteration
		return;
	}

	if ((sensor_retry_start - HAL_GetTick()) > SENSOR_RETRY) {
		if (!check_sensors_connected(&(CC_heartbeatState.errorFlags))) {
			// sensor failed

			sensor_retry_start = HAL_GetTick();
		}
		else {
			// sensors good, so setup filters and move to board check
			setup_adc_sensors();

			// disable timeout
			sensor_timeout_start = 0;

			fsm_changeState(fsm, &state_boardCheck, "Sensors initialized");
			return;
		}
	}

	if ((sensor_timeout_start - HAL_GetTick()) > SENSOR_TIMEOUT) {
		// sensor has failed so go to error state
		fsm_changeState(fsm, &state_error, "Sensors failed");
	}
}

void state_boardCheck_enter(fsm_t *fsm) {

}

void state_boardCheck_body(fsm_t *fsm) {
	// if all heartbeats / boards are present go to check AMS
}

void state_checkAMS_enter(fsm_t *fsm) {

}

void state_checkAMS_body(fsm_t *fsm) {
	// check AMS in ready state, if so go to idle
}

void state_error_enter(fsm_t *fsm) {

}

void state_error_body(fsm_t *fsm) {
	// do nothing
}


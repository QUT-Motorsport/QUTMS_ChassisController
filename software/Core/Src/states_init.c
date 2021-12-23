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

uint32_t peripheral_timeout_start = 0;
uint32_t peripheral_retry_start = 0;

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
		fsm_changeState(fsm, &state_pInit, "Peripherals initialized");
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
			fsm_changeState(fsm, &state_pInit, "Peripherals initialized");
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
	// init pedals

	// init steering angle

	// if both good go to board check
}

void state_sInit_body(fsm_t *fsm) {
	// if pedals / steering angle not connected, check again

	// if critical sensor not working > 5 attempts, go to error
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


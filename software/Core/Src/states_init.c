/*
 * CC_states_init.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"
#include "can_dict.h"

state_t state_start = { &state_start_enter, &state_start_body, CC_STATE_START };
state_t state_pInit = { &state_pInit_enter, &state_pInit_body, CC_STATE_PERIPHERAL_INIT };
state_t state_sInit = { &state_sInit_enter, &state_sInit_body, CC_STATE_SENSOR_INIT };
state_t state_boardCheck = { &state_boardCheck_enter, &state_boardCheck_body, CC_STATE_BOARD_CHECK };
state_t state_checkAMS = { &state_checkAMS_enter, &state_checkAMS_body, CC_STATE_AMS_CHECK };
state_t state_error = { &state_error_enter, &state_error_body, CC_STATE_ERROR };

void state_start_enter(fsm_t *fsm) {
	// init object dictionary
	CC_OD_init();

	// go to peripheral init
	fsm_changeState(fsm, &state_pInit, "Init Peripherals");
}

void state_start_body(fsm_t *fsm) {
	// do nothing?
}

void state_pInit_enter(fsm_t *fsm) {
	// setup CAN
}

void state_pInit_body(fsm_t *fsm) {
	// if CAN setup, go to sensor init

	// else try to reinit can

	// if haven't worked in 1000ms, go to error
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


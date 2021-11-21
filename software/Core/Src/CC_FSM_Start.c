/*
 * CC_FSM_Start.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "CC_FSM_States.h"
#include "main.h"
#include <FSM.h>
#include "RTD.h"

state_t deadState = { &state_dead_enter, &state_dead_iterate, &state_dead_exit,
		"Dead_s" };

void state_dead_enter(fsm_t *fsm) {
	debugCAN_enterState(CC_STATE_ID_Dead);
	return;
}

void state_dead_iterate(fsm_t *fsm) {
	return;
}

void state_dead_exit(fsm_t *fsm) {
	debugCAN_exitState(CC_STATE_ID_Dead);
	return;
}

state_t startState = { &state_start_enter, &state_start_iterate,
		&state_start_exit, "Start_s" };

void state_start_enter(fsm_t *fsm) {
	debugCAN_enterState(CC_STATE_ID_Start);

	RTD_state.shutdown_fault = false;

	// set initial pin states
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
			GPIO_PIN_RESET);

	return;
}

void state_start_iterate(fsm_t *fsm) {

	// pdm checks go here

	// go to idle
	fsm_changeState(fsm, &idleState, "Entering Idle");

	return;
}

void state_start_exit(fsm_t *fsm) {
	debugCAN_exitState(CC_STATE_ID_Start);

	RTD_state.precharge_enabled = false;
	RTD_state.precharge_done = false;

	return;
}

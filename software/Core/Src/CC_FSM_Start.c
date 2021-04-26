/*
 * CC_FSM_Start.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "CC_FSM_States.h"

state_t deadState = { &state_dead_enter, &state_dead_iterate, &state_dead_exit,
		"Dead_s" };

void state_dead_enter(fsm_t *fsm) {
	return;
}

void state_dead_iterate(fsm_t *fsm) {
	return;
}

void state_dead_exit(fsm_t *fsm) {
	return;
}

state_t startState = { &state_start_enter, &state_start_iterate, &state_start_exit,
		"Start_s" };

void state_start_enter(fsm_t *fsm) {
	return;
}

void state_start_iterate(fsm_t *fsm) {
	return;
}

void state_start_exit(fsm_t *fsm) {
	return;
}

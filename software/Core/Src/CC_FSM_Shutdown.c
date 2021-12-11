/*
 * CC_FSM_Shutdown.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include <FSM.h>
#include "inverter.h"
#include "CC_FSM_States.h"
#include "debugCAN.h"

state_t shutdownState = { &state_shutdown_enter, &state_shutdown_iterate,
		&state_shutdown_exit, "Shutdown_s" };

void state_shutdown_enter(fsm_t *fsm) {
	debugCAN_enterState(CC_STATE_ID_Shutdown);

	// whenever we enter shutdown kill the inverters again just to be safe
	inverter_send_shutdown();
}

void state_shutdown_iterate(fsm_t *fsm) {

}

void state_shutdown_exit(fsm_t *fsm) {
	debugCAN_exitState(CC_STATE_ID_Shutdown);
}


/*
 * CC_FSM_Shutdown.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include <FSM.h>
#include "inverter_roboteq.h"
#include "CC_FSM_States.h"

state_t shutdownState = { &state_shutdown_enter, &state_shutdown_iterate,
		&state_shutdown_exit, "Shutdown_s" };

void state_shutdown_enter(fsm_t *fsm) {
	// whenever we enter shutdown kill the inverters again just to be safe
	roboteq_send_shutdown();
}

void state_shutdown_iterate(fsm_t *fsm) {

}

void state_shutdown_exit(fsm_t *fsm) {

}


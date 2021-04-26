/*
 * CC_FSM_Driving.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */


#include <QUTMS_can.h>
#include "CC_FSM_States.h"
#include "heartbeat.h"
#include <FSM.h>
#include <Timer.h>
#include <stdbool.h>

#include "pedal_adc.h"
#include "inverter_roboteq.h"

state_t drivingState = { &state_driving_enter, &state_driving_iterate,
		&state_driving_exit, "Driving_s" };

ms_timer_t timer_inverters;
void inverter_timer_cb(void *args);

void state_driving_enter(fsm_t *fsm) {
	// send enable to inverters
	roboteq_update_enabled(true);

	// update duty cycle on fans

	// start inverter timer
	timer_inverters = timer_init(5, true, inverter_timer_cb);

	timer_start(&timer_inverters);
}

void state_driving_iterate(fsm_t *fsm) {
	// check CAN messages

	CAN_MSG_Generic_t msg;

	// CAN2
	while (queue_next(&queue_CAN2, &msg)) {
		if (msg.ID == AMS_HeartbeatResponse_ID) {
			heartbeats.AMS = true;
		} else if (msg.ID == SHDN_ShutdownTriggered_ID) {
			// send shutdown to inverters
			roboteq_send_shutdown();

			// change to error state
			fsm_changeState(fsm, &shutdownState, "Fatal Shutdown");
		}
	}

	// CAN1
	while (queue_next(&queue_CAN1, &msg)) {

	}

	// CAN3
	while (queue_next(&queue_CAN3, &msg)) {

	}

	// check heartbeats


	// update brake light


	// send pedal values to inverters
	timer_update(&timer_inverters, fsm);
}

void state_driving_exit(fsm_t *fsm) {
	// only exit here for either soft shutdown (pedals out of sync)
	// or big shutdown -> power is going off

	timer_stop(&timer_inverters);
}

void inverter_timer_cb(void *args) {
	// calculate APPS
	if (false) {
		// apps failed

		// turn off inverters nicely
		roboteq_update_enabled(false);

		// disable last stage of rtd so they have to press button again

		// go back to idle
		fsm_changeState((fsm_t *)args, &idleState, "Apps Shutdown");
	}

	// resend enabled to inverters
	roboteq_update_enabled(true);

	// send pedal values to inverters
	roboteq_send_pedals(current_pedal_values.pedal_accel_mapped[0], current_pedal_values.pedal_brake_mapped);

}

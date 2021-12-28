/*
 * shutdown.c
 *
 *  Created on: 28 Dec. 2021
 *      Author: Calvin
 */

#include "shutdown.h"

#include <CAN_VCU.h>

#include "states.h"
#include "can.h"
#include "heartbeat.h"
#include "RTD.h"

state_t state_shutdown = { &state_shutdown_enter, &state_shutdown_body, CC_STATE_SHUTDOWN };

bool shutdown_triggered;

void state_shutdown_enter(fsm_t *fsm) {
	RTD_BTN_Off();
}

void state_shutdown_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	bool shutdown_status = false;

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeats
		if (!check_heartbeat_msg(&msg)) {
			if ((msg.ID & ~0xF) == VCU_ShutdownStatus_ID) {
				uint8_t line;
				bool status;
				Parse_VCU_ShutdownStatus(msg.data, &line, &line, &line, &line, &status);

				shutdown_status = status;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

	if (shutdown_status) {
		// shutdown is good now, go back to AMS health check
		fsm_changeState(fsm, &state_checkAMS, "Shutdown fixed");
		return;
	}
}

bool check_shutdown_msg(CAN_MSG_Generic_t *msg, bool *shdn_triggered) {
	bool has_msg = false;
	*shdn_triggered = false;

	if ((msg->ID & ~0xF) == VCU_ShutdownStatus_ID) {
		has_msg = true;

		uint8_t line;
		bool status;
		Parse_VCU_ShutdownStatus(msg->data, &line, &line, &line, &line, &status);

		if (!status) {
			// shutdown triggered
			*shdn_triggered = true;
		}
	}
	else if (msg->ID == SHDN_ShutdownTriggered_ID) {
		has_msg = true;

		// shutdown triggered
		*shdn_triggered = true;
	}
	else if (msg->ID == AMS_ShutdownTriggered_ID) {
		has_msg = true;

		// AMS shutdown triggered
		*shdn_triggered = true;
	}

	return has_msg;
}

/*
 * shutdown.c
 *
 *  Created on: 28 Dec. 2021
 *      Author: Calvin
 */

#include "shutdown.h"

#include <CAN_VCU.h>

#include "states.h"

bool shutdown_triggered;

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

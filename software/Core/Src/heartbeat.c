/*
 * heartbeat.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "heartbeat.h"
#include "can.h"

heatbeat_states_t heartbeats;

AMS_HeartbeatState_t AMS_heartbeatState;
CC_HeartbeatState_t CC_heartbeatState;
MCISO_HeartbeatState_t MCISO_heartbeatState[MCISO_COUNT];

ms_timer_t timer_heartbeat;
uint32_t heartbeat_print_timer_start;

void setup_heartbeat() {
	// send heartbeat every 100ms
	timer_heartbeat = timer_init(100, true, heartbeat_timer_cb);

	// setup constants
	heartbeats.heartbeat_timeout = HEARTBEAT_TIMEOUT;

	// reset heartbeat timers to default
	heartbeat_timeout_reset();

	// start timer
	timer_start(&timer_heartbeat);

	heartbeat_print_timer_start = HAL_GetTick();
}

void heartbeat_timer_cb(void *args) {
	CC_Heartbeat_t msg = Compose_CC_Heartbeat(&CC_heartbeatState);
	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE };

	// send heartbeat on all CAN lines
	CC_send_can_msg(&hcan1, &header, msg.data);
	CC_send_can_msg(&hcan2, &header, msg.data);
	CC_send_can_msg(&hcan3, &header, msg.data);

	if ((HAL_GetTick() - heartbeat_print_timer_start) > HEARTBEAT_PRINT_TIME) {
		heartbeat_print_timer_start = HAL_GetTick();

		printf("HB: State: 0x%02X, Flags: 0x%04X, AMS: %d, MC0: %d, MC1: %d\r\n", CC_heartbeatState.stateID, CC_heartbeatState.errorFlags.rawMem, heartbeats.AMS ? 1 : 0, heartbeats.MCISO[0] ? 1 : 0, heartbeats.MCISO[1] ? 1 : 0);
	}
}

void heartbeat_timeout_reset() {
	heartbeats.hb_AMS_start = HAL_GetTick();

	for (int i = 0; i < MCISO_COUNT; i++) {
		heartbeats.hb_MCISO_start[i] = HAL_GetTick();
	}
}

bool check_heartbeat_msg(CAN_MSG_Generic_t *msg) {
	bool hb_message = false;

	uint8_t idx = (msg->ID & 0xF);
	uint32_t masked_id = (msg->ID & ~0xF);

	if (masked_id == MCISO_Heartbeat_ID) {
		hb_message = true;

		// check index is in correct range
		if (idx < MCISO_COUNT) {
			heartbeats.hb_MCISO_start[idx] = HAL_GetTick();
			heartbeats.MCISO[idx] = true;

			// update heartbeat struct
			Parse_MCISO_Heartbeat(msg->data, &MCISO_heartbeatState[idx]);
		}
	}
	else if (masked_id == AMS_Heartbeat_ID) {
		hb_message = true;

		heartbeats.hb_AMS_start = HAL_GetTick();
		heartbeats.AMS = true;

		// have heartbeat so clear error flag if it's set
		CC_heartbeatState.errorFlags.HB_AMS = 0;

		Parse_AMS_Heartbeat(msg->data, &AMS_heartbeatState);
	}

	// check all MCISO boards for valid heartbeats
	bool mciso_hb_good = true;
	for (int i = 0; i < MCISO_COUNT; i++) {
		if (!heartbeats.MCISO[i]) {
			mciso_hb_good = false;
			break;
		}
	}

	if (mciso_hb_good) {
		// valid heartbeats for all MCISO boards so clear error flag if it's set
		CC_heartbeatState.errorFlags.HB_MCISO = 0;
	}

	return hb_message;
}

bool check_bad_heartbeat() {
	bool success = true;

	// AMS
	if ((HAL_GetTick() - heartbeats.hb_AMS_start) > heartbeats.heartbeat_timeout) {
		heartbeats.AMS = false;
		CC_heartbeatState.errorFlags.HB_AMS = 1;
		success = false;
	}

	// MCISO
	for (int i = 0; i < MCISO_COUNT; i++) {
		if ((HAL_GetTick() - heartbeats.hb_MCISO_start[i]) > heartbeats.heartbeat_timeout) {
			heartbeats.MCISO[i] = false;
			CC_heartbeatState.errorFlags.HB_MCISO = 1;
			success = false;
		}
	}

	return success;
}

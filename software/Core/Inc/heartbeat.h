/*
 * heartbeat.h
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#ifndef INC_HEARTBEAT_H_
#define INC_HEARTBEAT_H_

#include <stdbool.h>
#include <CAN_CC.h>
#include <CAN_MCISO.h>
#include <Timer.h>

typedef struct heartbeat_states {
	uint32_t heartbeat_timeout;

	bool AMS;
	bool MCISO[MCISO_COUNT];

	uint32_t hb_AMS_start;
	uint32_t hb_MCISO_start[MCISO_COUNT];
} heatbeat_states_t;

extern heatbeat_states_t heartbeats;
extern ms_timer_t timer_heartbeat;

extern CC_HeartbeatState_t CC_heartbeatState;
extern MCISO_HeartbeatState_t MCISO_heartbeatState[MCISO_COUNT];

void setup_heartbeat();
void heartbeat_timer_cb(void *args);

void heartbeat_timeout_reset();

// call every time checking CAN message queue to update heartbeat status of boards
bool check_heartbeat_msg(CAN_MSG_Generic_t *msg);

// call to update status of heartbeat timeouts and detect potential board loss
bool check_bad_heartbeat();

#endif /* INC_HEARTBEAT_H_ */

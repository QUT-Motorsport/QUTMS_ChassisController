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

typedef struct heartbeat_states {
	bool AMS;
} heatbeat_states_t;

extern heatbeat_states_t heartbeats;

extern CC_HeartbeatState_t CC_heartbeatState;
extern MCISO_HeartbeatState_t MCISO_heartbeatState;

#endif /* INC_HEARTBEAT_H_ */

/*
 * heartbeat.h
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#ifndef INC_HEARTBEAT_H_
#define INC_HEARTBEAT_H_

#include <stdbool.h>

typedef struct heartbeat_states {
	bool AMS;
} heatbeat_states_t;

extern heatbeat_states_t heartbeats;

#endif /* INC_HEARTBEAT_H_ */

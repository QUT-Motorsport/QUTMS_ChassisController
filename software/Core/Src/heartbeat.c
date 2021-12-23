/*
 * heartbeat.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "heartbeat.h"

#include <stdbool.h>

heatbeat_states_t heartbeats;

CC_HeartbeatState_t CC_heartbeatState;
MCISO_HeartbeatState_t MCISO_heartbeatState;

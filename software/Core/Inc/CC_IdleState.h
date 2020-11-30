/*
 * CC_IdleState.h
 *
 *  Created on: Nov 30, 2020
 *      Author: calvi
 */

#ifndef INC_CC_IDLESTATE_H_
#define INC_CC_IDLESTATE_H_

#include "FSM.h"

void thread_idle_read_CAN(void *argument);


/**
 * @brief Idle state enter function. Initialises the CC_GlobalState
 * @param fsm A pointer to the FSM object
 */
void state_idle_enter(fsm_t *fsm);

/**
 * @brief Idle state iterate function. Monitor System Status
 * @param fsm A pointer to the FSM object
 */
void state_idle_iterate(fsm_t *fsm);

/**
 * @brief Idle state exit function. Broadcast RTD Message and execute Startup Sequence
 * @param fsm A pointer to the FSM object
 */
void state_idle_exit(fsm_t *fsm);

/**
 * @brief idleState ie. idle state before RTD request is received
 * @note Idle FSM state, waiting for RTD or charging CAN messages.
 * * @details Next: drivingState (RTD Engaged)
 */
state_t idleState;

#endif /* INC_CC_IDLESTATE_H_ */

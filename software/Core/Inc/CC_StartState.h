/*
 * CC_StartState.h
 *
 *  Created on: Nov 30, 2020
 *      Author: Calvin Johnson
 */

#ifndef INC_CC_STARTSTATE_H_
#define INC_CC_STARTSTATE_H_

#include "FSM.h"

/**
 * @brief Start state enter function. Initialises the CC_GlobalState
 * @param fsm A pointer to the FSM object
 */
void state_start_enter(fsm_t *fsm);

/**
 * @brief Start state iterate function. Execute boot sequence
 * @param fsm A pointer to the FSM object
 */
void state_start_iterate(fsm_t *fsm);

/**
 * @brief Start state exit function.
 * @param fsm A pointer to the FSM object
 */
void state_start_exit(fsm_t *fsm);

/**
 * @brief startState ie. start state before boot sequence is executed
 * @note LV System engaged
 * * @details Next: idleState (Boot executed)
 */
state_t startState;



#endif /* INC_CC_STARTSTATE_H_ */

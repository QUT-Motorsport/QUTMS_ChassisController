/*
 * CC_FSM_States.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */

#ifndef INC_CC_FSM_STATES_H_
#define INC_CC_FSM_STATES_H_

#include <FSM.h>

#define CC_STATE_ID_Dead 		0x0
#define CC_STATE_ID_Reset 		0x1
#define CC_STATE_ID_Start 		0x2
#define CC_STATE_ID_Idle 		0x4
#define CC_STATE_ID_Driving		0x6
#define CC_STATE_ID_Shutdown 	0x8

void state_dead_enter(fsm_t *fsm);
void state_dead_iterate(fsm_t *fsm);
void state_dead_exit(fsm_t *fsm);

extern state_t deadState;

void state_start_enter(fsm_t *fsm);
void state_start_iterate(fsm_t *fsm);
void state_start_exit(fsm_t *fsm);

extern state_t startState;

void state_idle_enter(fsm_t *fsm);
void state_idle_iterate(fsm_t *fsm);
void state_idle_exit(fsm_t *fsm);

extern state_t idleState;

void state_driving_enter(fsm_t *fsm);
void state_driving_iterate(fsm_t *fsm);
void state_driving_exit(fsm_t *fsm);

extern state_t drivingState;

void state_shutdown_enter(fsm_t *fsm);
void state_shutdown_iterate(fsm_t *fsm);
void state_shutdown_exit(fsm_t *fsm);

extern state_t shutdownState;

#endif /* INC_CC_FSM_STATES_H_ */

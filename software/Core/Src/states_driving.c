/*
 * states_driving.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"

state_t state_driving;
state_t state_tsError;

void state_driving_enter(fsm_t *fsm);
void state_driving_body(fsm_t *fsm);


void state_tsError_enter(fsm_t *fsm);
void state_tsError_body(fsm_t *fsm);


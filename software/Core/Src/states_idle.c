/*
 * states_idle.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"

void state_idle_enter(fsm_t *fsm);
void state_idle_body(fsm_t *fsm);
state_t state_idle;

void state_precharge_enter(fsm_t *fsm);
void state_precharge_body(fsm_t *fsm);
state_t state_precharge;

void state_checkInverter_enter(fsm_t *fsm);
void state_checkInverter_body(fsm_t *fsm);
state_t state_checkInverter;

void state_rtdReady_enter(fsm_t *fsm);
void state_rtdReady_body(fsm_t *fsm);
state_t state_rtdReady;

void state_rtdButton_enter(fsm_t *fsm);
void state_rtdButton_body(fsm_t *fsm);
state_t state_rtdButton;

void state_shutdown_enter(fsm_t *fsm);
void state_shutdown_body(fsm_t *fsm);
extern state_t state_shutdown;

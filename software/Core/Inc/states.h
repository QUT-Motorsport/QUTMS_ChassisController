/*
 * CC_states.h
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#ifndef INC_STATES_H_
#define INC_STATES_H_

#include <FSM.h>
#include <CAN_CC.h>

void state_start_enter(fsm_t *fsm);
void state_start_body(fsm_t *fsm);
extern state_t state_start;

void state_pInit_enter(fsm_t *fsm);
void state_pInit_body(fsm_t *fsm);
extern state_t state_pInit;

void state_sInit_enter(fsm_t *fsm);
void state_sInit_body(fsm_t *fsm);
extern state_t state_sInit;

void state_boardCheck_enter(fsm_t *fsm);
void state_boardCheck_body(fsm_t *fsm);
extern state_t state_boardCheck;

void state_checkAMS_enter(fsm_t *fsm);
void state_checkAMS_body(fsm_t *fsm);
extern state_t state_checkAMS;

void state_idle_enter(fsm_t *fsm);
void state_idle_body(fsm_t *fsm);
extern state_t state_idle;

void state_request_pchrg_enter(fsm_t *fsm);
void state_request_pchrg_body(fsm_t *fsm);
extern state_t state_request_pchrg;

void state_precharge_enter(fsm_t *fsm);
void state_precharge_body(fsm_t *fsm);
extern state_t state_precharge;

void state_checkInverter_enter(fsm_t *fsm);
void state_checkInverter_body(fsm_t *fsm);
extern state_t state_checkInverter;

void state_rtdReady_enter(fsm_t *fsm);
void state_rtdReady_body(fsm_t *fsm);
extern state_t state_rtdReady;

void state_rtdButton_enter(fsm_t *fsm);
void state_rtdButton_body(fsm_t *fsm);
extern state_t state_rtdButton;

void state_driving_enter(fsm_t *fsm);
void state_driving_body(fsm_t *fsm);
extern state_t state_driving;

void state_shutdown_enter(fsm_t *fsm);
void state_shutdown_body(fsm_t *fsm);
extern state_t state_shutdown;

void state_tsError_enter(fsm_t *fsm);
void state_tsError_body(fsm_t *fsm);
extern state_t state_tsError;

void state_error_enter(fsm_t *fsm);
void state_error_body(fsm_t *fsm);
extern state_t state_error;


#endif /* INC_STATES_H_ */

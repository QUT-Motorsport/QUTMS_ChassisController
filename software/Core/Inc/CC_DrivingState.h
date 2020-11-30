/*
 * CC_DrivingState.h
 *
 *  Created on: Nov 30, 2020
 *      Author: calvi
 */

#ifndef INC_CC_DRIVINGSTATE_H_
#define INC_CC_DRIVINGSTATE_H_

#include "FSM.h"

// 1.5 seconds
#define RTD_SIREN_TICKS 1500

// 100ms
#define IMPLAUSIBILITY_TICK_COUNT 100

#define NUM_INVERTERS 2
#define INVERTER_VAR_ACCEL 0x01
#define INVERTER_VAR_BRAKE 0x02

#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

extern const uint16_t inverter_node_ids[NUM_INVERTERS];

void thread_driving_update_pdm(void *argument);
void thread_driving_update_inverters(void *argument);
void thread_driving_read_CAN(void *argument);

/**
 * Driving state enter function. Ensure heartbeat integrity before engaging tractive system
 * @param fsm A pointer to the FSM object
 */
void state_driving_enter(fsm_t *fsm);

/**
 * Driving state iterate function. Send torque commands, monitor heartbeats and monitor pedal plausibility
 * @param fsm A pointer to the FSM object
 */
void state_driving_iterate(fsm_t *fsm);

/**
 * Driving state exit function. Broadcast soft shutdown state globally
 * @param fsm A pointer to the FSM object
 */
void state_driving_exit(fsm_t *fsm);

/**
 * @brief drivingState for tractive system operational
 * @note Driving FSM state
 * @details Next: idleState (Soft Shutdown)
 */
state_t drivingState;

#endif /* INC_CC_DRIVINGSTATE_H_ */

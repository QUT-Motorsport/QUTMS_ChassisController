/**
 ******************************************************************************
 * @file CC_FSM_States.h
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#ifndef INC_CC_FSM_STATES_H_
#define INC_CC_FSM_STATES_H_

#include "FSM.h"
#include "main.h"
#include "cmsis_os.h"
#include <memory.h>
#include <stdbool.h>
#include "can.h"

/**
 * @brief Chassis Global State
 * @note Chassis Global State is shared across threads, so use the semaphore to gain control
 */
typedef struct
{
  //CAN
  uint32_t CAN1_TxMailbox;
  uint32_t CAN1_RxMailbox;
  uint32_t CAN2_TxMailbox;
  uint32_t CAN2_RxMailbox;
  uint32_t CAN3_TxMailbox;
  uint32_t CAN3_RxMailbox;

  uint32_t startupTicks; /**< The Tick count at the initial startup time */

  osMessageQueueId_t CANQueue;
  osTimerId_t heartbeatTimer;
  osTimerId_t IDC_AlarmTimer;
  osSemaphoreId_t sem;
} CC_GlobalState_t;

CC_GlobalState_t *CC_GlobalState;

/**
 * @brief Dead state enter function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_enter(fsm_t *fsm);

/**
 * @brief Dead state iterate function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_iterate(fsm_t *fsm);

/**
 * @brief Dead state exit function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_exit(fsm_t *fsm);

/**
 * @brief deadState ie. startup state for the fsm
 * @note Initial FSM state, has no functionality
 * @details Next: idleState (Instantly)
 */
state_t deadState;

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

#endif /* INC_CC_FSM_STATES_H_ */

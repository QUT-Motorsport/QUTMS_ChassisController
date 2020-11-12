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
#include "usart.h"
#include "adc.h"
#include "CC_CAN_Messages.h"
#include "CC_CAN_Wrapper.h"
#include "PDM_CAN_Messages.h"
#include "AMS_CAN_Messages.h"
#include "SHDN_IMD_CAN_Messages.h"
#include "SHDN_CAN_Messages.h"
#include "Util.h"

/**
 * @brief Chassis Global State
 * @note Chassis Global State is shared across threads, so use the semaphore to gain control
 */
typedef struct
{
	/* CAN Mailboxes */
	uint32_t CAN1_TxMailbox;
	uint32_t CAN1_RxMailbox;
	uint32_t CAN2_TxMailbox;
	uint32_t CAN2_RxMailbox;
	uint32_t CAN3_TxMailbox;
	uint32_t CAN3_RxMailbox;

	/* Debugger for board connectivity
	 * true = Board not connected
	 * false = Board connected
	 */
	bool RTD_Debug;
	bool ADC_Debug;

	bool PDM_Debug;
	bool AMS_Debug;
	bool Inverter_Debug;
	bool SHDN_IMD_Debug;
	bool SHDN_Debug;

	/** Tick Refresh Counter for Individual Board Heartbeats */
	uint32_t startupTicks;
	uint32_t readyToDriveTicks;
	uint32_t implausibleTicks;
	uint32_t amsTicks;
	uint32_t inverterTicks;
	uint32_t shutdownTicks;
	uint32_t shutdownImdTicks;

	/* Initialisation Confirmation */
	bool ccInit;
	bool amsInit;

	/* Analogue Values */
	uint32_t brakeAdcValues[100];
	uint32_t accelAdcValues[150];
	uint32_t brakePressureThreshold;
	uint16_t rollingBrakeValues[10]; uint16_t secondaryRollingBrakeValues[10];
	uint16_t brakeMin[2]; uint16_t brakeMax[2];
	uint16_t accelMin[3]; uint16_t accelMax[3];
	uint32_t rollingAccelValues[10]; uint32_t secondaryRollingAccelValues[10]; uint32_t tertiaryRollingAccelValues[10];

	/* Formatted Pedal Travel Positions */
	uint8_t accelTravel;
	uint8_t brakeTravel;

	bool tractiveActive;
	bool faultDetected;
	bool rtdLightActive;

	osMessageQueueId_t CAN1Queue;
	osMessageQueueId_t CAN2Queue;
	osMessageQueueId_t CAN3Queue;
	osTimerId_t heartbeatTimer;
	osTimerId_t IDC_AlarmTimer;
	osSemaphoreId_t sem;

	uint32_t rtdTicks;
	uint32_t rtdTicksSpan;
	uint32_t finalRtdTicks;
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

/**
 * Debug state enter function.
 * @param fsm A pointer to the FSM object
 */
void state_debug_enter(fsm_t *fsm);

/**
 * Debug state iterate function. Spit CAN out
 * @param fsm A pointer to the FSM object
 */
void state_debug_iterate(fsm_t *fsm);

/**
 * Debug state exit function
 * @param fsm A pointer to the FSM object
 */
void state_debug_exit(fsm_t *fsm);

/**
 * @brief debugState for debugging functionality
 * @note Debugging FSM state
 */
state_t debugState;

#endif /* INC_CC_FSM_STATES_H_ */

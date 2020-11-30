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

// states
#include "CC_StartState.h"
#include "CC_IdleState.h"
#include "CC_DrivingState.h"

#define NUM_BRAKE_SENSORS 2
#define NUM_ACCEL_SENSORS 3


#define BRAKE_PEDAL_ONE_MIN 2350
#define BRAKE_PEDAL_ONE_MAX 3250
#define BRAKE_PEDAL_TWO_MIN 2320
#define BRAKE_PEDAL_TWO_MAX 3100

#define ACCEL_PEDAL_ONE_MIN 2450
#define ACCEL_PEDAL_ONE_MAX 3300
#define ACCEL_PEDAL_TWO_MIN 2800
#define ACCEL_PEDAL_TWO_MAX 3300
#define ACCEL_PEDAL_THREE_MIN 2800
#define ACCEL_PEDAL_THREE_MAX 3300

typedef struct CC_CAN_State {
	/* CAN Mailboxes */
	uint32_t CAN1_TxMailbox;
	uint32_t CAN1_RxMailbox;
	uint32_t CAN2_TxMailbox;
	uint32_t CAN2_RxMailbox;
	uint32_t CAN3_TxMailbox;
	uint32_t CAN3_RxMailbox;

	// message queues to store CAN messages
	osMessageQueueId_t CAN1Queue;
	osMessageQueueId_t CAN2Queue;
	osMessageQueueId_t CAN3Queue;

	// semaphore for thread protection
	osSemaphoreId_t sem;

} CC_CAN_State_t;

typedef struct CC_Tractive_State {
	uint16_t accel_pedals_raw[NUM_ACCEL_SENSORS];
	uint16_t brake_pedals_raw[NUM_BRAKE_SENSORS];

	uint16_t accel_pedals[NUM_ACCEL_SENSORS];
	uint16_t brake_pedals[NUM_BRAKE_SENSORS];

	uint16_t accel_value;
	uint16_t brake_value;

	uint16_t accel_min[NUM_ACCEL_SENSORS];
	uint16_t accel_max[NUM_ACCEL_SENSORS];

	uint16_t brake_min[NUM_BRAKE_SENSORS];
	uint16_t brake_max[NUM_BRAKE_SENSORS];

	uint32_t rtd_ticks;

	uint32_t implausible_ticks;
	bool fault_detected;

	bool tractive_active;

	// semaphore for thread protection
	osSemaphoreId_t sem;
} CC_Tractive_State_t;

typedef struct CC_Heartbeat_State {
	// tick count between heartbeats
	uint32_t amsTicks;
	uint32_t inverterTicks;
	uint32_t shutdownOneTicks;
	uint32_t shutdownTwoTicks;
	uint32_t shutdownThreeTicks;
	uint32_t shutdownImdTicks;

	// semaphore for thread protection
	osSemaphoreId_t sem;

} CC_Heartbeat_State_t;

typedef struct CC_Global_State {
	// debug values to disable checks for debugging
	bool ADC_Debug;
	bool PDM_Debug;
	bool AMS_Debug;
	bool Inverter_Debug;
	bool SHDN_IMD_Debug;
	bool SHDN_1_Debug;
	bool SHDN_2_Debug;
	bool SHDN_3_Debug;

	uint32_t pdm_channel_states;

	uint32_t rtd_ticks;

	bool CC_initialized;
	bool AMS_initialized;

	bool shutdown_fault;

	// semaphore for thread protection
	osSemaphoreId_t sem;

} CC_Global_State_t;

void thread_read_pedals(void *argument);
void thread_check_heartbeats(void *argument);

typedef struct CC_main_threads {
	osThreadId_t main_read_pedals_handle;
	osThreadId_t main_check_heartbeats_handle;

} CC_main_threads_t;

/**
 * @brief Chassis Global State
 * @note Chassis Global State is shared across threads, so use the semaphore to gain control
 */
/*
typedef struct {

	//	Debugger for board connectivity
	//	true = Board not connected
	//	false = Board connected
	bool RTD_Debug;
	bool ADC_Debug;

	bool PDM_Debug;
	bool AMS_Debug;
	bool Inverter_Debug;
	bool SHDN_IMD_Debug;
	bool SHDN_1_Debug;
	bool SHDN_2_Debug;
	bool SHDN_3_Debug;

	//Tick Refresh Counter for Individual Board Heartbeats
	uint32_t startupTicks;
	uint32_t readyToDriveTicks;
	uint32_t implausibleTicks;
	uint32_t amsTicks;
	uint32_t inverterTicks;
	uint32_t shutdownOneTicks;
	uint32_t shutdownTwoTicks;
	uint32_t shutdownThreeTicks;
	uint32_t shutdownImdTicks;

	// PDM Channel Management
	uint32_t pdmTrackState;

	// Initialisation Confirmation
	bool ccInit;
	bool amsInit;

	// Analogue Values
	uint32_t brakeAdcValues[100];
	uint32_t accelAdcValues[150];
	uint32_t brakePressureThreshold;
	uint32_t rollingBrakeValues[10];
	uint32_t secondaryRollingBrakeValues[10];
	uint32_t brakeMin[2];
	uint32_t brakeMax[2];
	uint32_t accelMin[3];
	uint32_t accelMax[3];
	uint32_t rollingAccelValues[10];
	uint32_t secondaryRollingAccelValues[10];
	uint32_t tertiaryRollingAccelValues[10];

	// Formatted Pedal Travel Positions
	int16_t accelTravel;
	int16_t brakeTravel;

	bool tractiveActive;
	bool faultDetected;
	bool rtdLightActive;

	bool shutdown_fault;

	osTimerId_t heartbeatTimer;
	osTimerId_t IDC_AlarmTimer;
	osSemaphoreId_t sem;

	uint32_t rtdTicks;
	uint32_t rtdTicksSpan;
	uint32_t finalRtdTicks;

	uint32_t inverter_cmd_ticks;
	uint32_t inverter_enable_ticks;

	uint8_t duty_cycle_left_fan;
	uint8_t duty_cycle_right_fan;

	uint32_t fan_cmd_ticks;
} CC_GlobalState_t;
*/

extern CC_Global_State_t *CC_Global_State;
extern CC_CAN_State_t *CC_CAN_State;
extern CC_Tractive_State_t *CC_Tractive_State;
extern CC_Heartbeat_State_t *CC_Heartbeat_State;

extern state_t startState;
extern state_t idleState;
extern state_t drivingState;

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

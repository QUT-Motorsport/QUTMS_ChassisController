/**
 ******************************************************************************
 * @file CC_FSM_States.c
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#include "CC_FSM_States.h"
#include "main.h"

#define HEARTBEAT_TIMEOUT 100

// 10%
#define PEDAL_IMPLAUSIBILITY 100



#define BRAKE_PRESSURE_MIN 400
#define BRAKE_PRESSURE_MAX 1400

#define DEAD_ZONE_BRAKE 60
#define DEAD_ZONE_ACCEL 150

#define POT_DESYNC 250

#define MAX_PEDAL_DUTY_CYCLE 1000
#define MAX_DUTY_CYCLE 1000

#define CAN_1 hcan1
#define CAN_2 hcan2
#define CAN_3 hcan3

#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

#define CC_MASK PDM_POWER_CC_MASK

#define INVERTER_CMD_TICK_COUNT 20
#define INVERTER_ENABLE_TICK_COUNT 15



void state_debug_exit(fsm_t *fsm) {
	//CC_LogInfo("Exit Debugging\r\n", strlen("Exit Debugging\r\n"));
	return;
}

void thread_read_pedals(void *argument) {
	uint32_t a_pedals[NUM_ACCEL_SENSORS];
	uint32_t b_pedals[NUM_BRAKE_SENSORS];

	// sample over 10 readings
	float n = 10;

	// start DMA of pedal ADC
	HAL_ADC_Start_DMA(&hadc1, a_pedals, NUM_ACCEL_SENSORS);
	HAL_ADC_Start_DMA(&hadc2, b_pedals, NUM_BRAKE_SENSORS);

	// small delay to get first set of readings
	osDelay(10);

	// setup initial readings
	int i = 0;
	if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		for (i = 0; i < NUM_ACCEL_SENSORS; i++) {
			CC_Tractive_State->accel_pedals_raw[i] = (uint16_t) a_pedals[i];
		}

		for (i = 0; i < NUM_BRAKE_SENSORS; i++) {
			CC_Tractive_State->brake_pedals_raw[i] = (uint16_t) b_pedals[i];
		}

		osSemaphoreRelease(CC_Tractive_State->sem);
	}

	for (;;) {
		if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			for (i = 0; i < NUM_ACCEL_SENSORS; i++) {
				// noise filtering
				CC_Tractive_State->accel_pedals_raw[i] += (a_pedals[i] - CC_Tractive_State->accel_pedals_raw[i]) / n;

				// map values
				CC_Tractive_State->accel_pedals[i] = map(CC_Tractive_State->accel_pedals_raw,
						CC_Tractive_State->accel_min[i], CC_Tractive_State->accel_max[i], 0, MAX_PEDAL_DUTY_CYCLE);

			}

			for (i = 0; i < NUM_BRAKE_SENSORS; i++) {
				// noise filtering
				CC_Tractive_State->brake_pedals_raw[i] += (b_pedals[i] - CC_Tractive_State->brake_pedals_raw[i]) / n;

				// map values
				CC_Tractive_State->brake_pedals[i] = map(CC_Tractive_State->brake_pedals_raw,
						CC_Tractive_State->brake_min[i], CC_Tractive_State->brake_max[i], 0, MAX_PEDAL_DUTY_CYCLE);
			}

			// calculate plausibility
			bool accel_fault = true;
			for (i = 0; i < NUM_ACCEL_SENSORS; i++) {
				// we're fine if any two match
				for (int j = 0; j < NUM_ACCEL_SENSORS; j++) {
					if (i != j) {
						if (abs(CC_Tractive_State->accel_pedals[i] - CC_Tractive_State->accel_pedals[j]) < PEDAL_IMPLAUSIBILITY) {
							accel_fault = false;
							// this value is fine, but invert it to fit our ADC reading
							CC_Tractive_State->accel_value = MAX_PEDAL_DUTY_CYCLE - CC_Tractive_State->accel_pedals[i];
							break;
						}
					}
				}
			}

			bool brake_fault = !(abs(CC_Tractive_State->brake_pedals[0] - CC_Tractive_State->brake_pedals[1]) < PEDAL_IMPLAUSIBILITY);
			if (!brake_fault) {
				// this value is fine, but invert it to fit our ADC reading
				CC_Tractive_State->brake_value = MAX_PEDAL_DUTY_CYCLE - CC_Tractive_State->brake_pedals[0];
			}

			bool current_fault = (brake_fault || accel_fault);

			if (current_fault) {
				if (!CC_Tractive_State->fault_detected) {
					// first instance of fault
					CC_Tractive_State->implausible_ticks = HAL_GetTicks();
				}
				CC_Tractive_State->fault_detected = true;
			}

			if (!current_fault && CC_Tractive_State->fault_detected) {
				// had a fault but don't currently
				CC_Tractive_State->fault_detected = false;
				CC_Tractive_State->implausible_ticks = 0;
			}

			// finally apply a deadzone so we have a minimum value
			if (CC_GlobalState->accelTravel < DEAD_ZONE_ACCEL) {
				CC_GlobalState->accelTravel = 0;
			}
			if (CC_GlobalState->brakeTravel < DEAD_ZONE_BRAKE) {
				CC_GlobalState->brakeTravel = 0;
			}

			osSemaphoreRelease(CC_Tractive_State->sem);
		}

		// run every 10ms
		osDelay(10);
	}

	// if we exit loop for some reason
	osThreadTerminate(NULL);
}

void thread_check_heartbeats(void *argument) {
	for (;;) {
		if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			/* AMS Heartbeat Expiry - Fatal Shutdown */
			if ((HAL_GetTick() - CC_Heartbeat_State->amsTicks) > HEARTBEAT_TIMEOUT && !CC_Heartbeat_State->AMS_Debug) {
				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
			}
			/* Shutdown Heartbeat Expiry - Fatal Shutdown */
			if ((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > HEARTBEAT_TIMEOUT
					&& !CC_GlobalState->SHDN_1_Debug) {
				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
			}

			/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
			if ((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > HEARTBEAT_TIMEOUT
					&& !CC_GlobalState->SHDN_IMD_Debug) {
				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
			}
			/* Inverter Heartbeat Expiry - Fatal Shutdown */
			if ((HAL_GetTick() - CC_GlobalState->inverterTicks) > HEARTBEAT_TIMEOUT
					&& !CC_GlobalState->Inverter_Debug) {
				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
			}
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		// run this every 50 ms
		osDelay(50);
	}

	// if we exit loop for some reason
	osThreadTerminate(NULL);
}

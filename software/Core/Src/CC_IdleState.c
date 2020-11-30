/*
 * CC_IdleState.c
 *
 *  Created on: Nov 30, 2020
 *      Author: calvi
 */

#include "CC_IdleState.h"

typedef struct CC_idle_threads {
	osThreadId_t idle_read_CAN_handle;

} CC_idle_threads_t;

CC_idle_threads_t *CC_idle_threads;



void thread_idle_read_CAN(void *argument) {

}

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s" };

void state_idle_enter(fsm_t *fsm) {
	if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

		// set LV PDM channels
		if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

			CC_Global_State->pdm_channel_states = LV_STARTUP | PDM_POWER_CC_MASK;

			PDM_SetChannelStates_t update_pdm_msg = Compose_PDM_SetChannelStates(CC_Global_State->pdm_channel_states);
			CAN_TxHeaderTypeDef CAN_header = { .ExtId = update_pdm_msg.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA,
					.DLC = sizeof(update_pdm_msg.data), .TransmitGlobalTime = DISABLE, };

			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, update_pdm_msg.data, &CC_CAN_State->CAN2_TxMailbox);

			osSemaphoreRelease(CC_CAN_State->sem);
		}

		/* Init Chassis Controller On */
		CC_GlobalState->ccInit = true;
		osSemaphoreRelease(CC_Global_State->sem);
	}

	// start idle specific threads



	return;
}

void state_idle_iterate(fsm_t *fsm) {
	/* Check for Heartbeat Expiry */
	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->amsTicks) > 100 && !CC_GlobalState->AMS_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 1 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100 && !CC_GlobalState->SHDN_1_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 2 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownTwoTicks) > 100 && !CC_GlobalState->SHDN_2_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN2\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 3 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownThreeTicks) > 100 && !CC_GlobalState->SHDN_3_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN3\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100 && !CC_GlobalState->Inverter_Debug) {
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
					&CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			CC_GlobalState->shutdown_fault = true;
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets on CAN1 */
	while (osMessageQueueGetCount(CC_GlobalState->CAN1Queue) >= 1) {
		CC_CAN_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN1Queue, &msg, 0U, 0U) == osOK) {
			if (msg.header.IDE == CAN_ID_STD) {
				/* Inverter Heartbeat */
				if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
			}
		}
	}

	/* Check for Queued CAN Packets on CAN2 */
	while (osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1) {
		CC_CAN_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK) {
			/* Packet Handler */
			/* AMS Heartbeat */
			if (msg.header.IDE == CAN_ID_EXT) {
				if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
					if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
						bool initialised = false;
						bool HVAn;
						bool HVBn;
						bool precharge;
						bool HVAp;
						bool HVBp;
						uint16_t averageVoltage;
						uint16_t runtime;
						Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn, &precharge, &HVAp, &HVBp,
								&averageVoltage, &runtime);
						CC_GlobalState->amsTicks = HAL_GetTick();
						CC_GlobalState->amsInit = initialised;
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 1 Heartbeat */
				else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0)) {
					//CC_LogInfo("SHDN1\r\n", strlen("SHDN1\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
						CC_GlobalState->shutdownOneTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x02, 0x0)) {
					//CC_LogInfo("SHDN2\r\n", strlen("SHDN2\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
						CC_GlobalState->shutdownTwoTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x03, 0x0)) {
					//CC_LogInfo("SHDN3\r\n", strlen("SHDN3\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
						CC_GlobalState->shutdownThreeTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown IMD Heartbeat */
				else if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
					uint8_t pwmState;
					Parse_SHDN_IMD_HeartbeatResponse(*((SHDN_IMD_HeartbeatResponse_t*) &(msg.data)), &pwmState);
					CC_GlobalState->shutdownImdTicks = HAL_GetTick();
				}
				/* Shutdown Triggered Fault */
				else if (msg.header.ExtId == Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0)) {
					// TODO DEAL WITH INVERTERS HERE WITH SOFT INVERTER SHUTDOWN
					CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault (idle)\r\n", true,
							&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
							&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3, INVERTER_LEFT_NODE_ID,
							INVERTER_RIGHT_NODE_ID);
					CC_GlobalState->shutdown_fault = true;
				}
			}

		}
	}

	fsm_changeState(fsm, &drivingState, "RTD Engaged");

	// don't let continue
	if (CC_GlobalState->shutdown_fault) {
		return;
	}

	/* If Brake Pressure > 20% */
	uint16_t raw;
	if (CC_GlobalState->RTD_Debug) {
		int brake_threshold_range = BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN;
		raw = BRAKE_PRESSURE_MIN + (0.3 * brake_threshold_range);
	} else {
		HAL_ADC_Start(&hadc3);
		raw = HAL_ADC_GetValue(&hadc3);
	}

	if (raw > CC_GlobalState->brakePressureThreshold && (CC_GlobalState->amsInit || CC_GlobalState->AMS_Debug)
			&& CC_GlobalState->ccInit) {
		// Illuminate RTD Button
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);
		// If RTD Button Engaged
		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin)
					&& (HAL_GetTick() - CC_GlobalState->finalRtdTicks) >= 5000) {
				// Enter Driving State
				fsm_changeState(fsm, &drivingState, "RTD Engaged");
			}
			osSemaphoreRelease(CC_GlobalState->sem);
		}
	} else {
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
	}

}

void state_idle_exit(fsm_t *fsm) {

	if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		// send RTD on all CAN lines
		CC_ReadyToDrive_t readyToDrive = Compose_CC_ReadyToDrive();
		CAN_TxHeaderTypeDef CAN_header = { .ExtId = readyToDrive.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1,
				.TransmitGlobalTime = DISABLE, };
		uint8_t data[1] = { 0xF };
		HAL_CAN_AddTxMessage(&hcan1, &CAN_header, data, &CC_CAN_State->CAN1_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan2, &CAN_header, data, &CC_CAN_State->CAN2_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan3, &CAN_header, data, &CC_CAN_State->CAN3_TxMailbox);

		// turn RTD siren on and enable HV power
		if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			CC_Global_State->pdm_channel_states = CC_Global_State->pdm_channel_states | PDMFLAG_RTD_SIREN | HV_STARTUP;

			PDM_SetChannelStates_t update_pdm_msg = Compose_PDM_SetChannelStates(CC_Global_State->pdm_channel_states);
			CAN_header.ExtId = update_pdm_msg.id;
			CAN_header.DLC = sizeof(update_pdm_msg.data);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, update_pdm_msg.data, &CC_CAN_State->CAN2_TxMailbox);

			osSemaphoreRelease(CC_Global_State->sem);

		}

		osSemaphoreRelease(CC_CAN_State->sem);
	}

	// kill idle specific rtos threads and free memory

	if (CC_idle_threads != NULL) {
		// terminate threads
		osThreadTerminate(CC_idle_threads->idle_read_CAN_handle);

		// free memory
		free(CC_idle_threads);
		CC_idle_threads = NULL;
	}

	return;
}

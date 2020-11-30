/*
 * CC_IdleState.c
 *
 *  Created on: Nov 30, 2020
 *      Author: calvi
 */

#include "CC_FSM_States.h"
#include "CC_IdleState.h"

typedef struct CC_idle_threads {
	osThreadId_t idle_read_CAN_handle;

} CC_idle_threads_t;

CC_idle_threads_t *CC_idle_threads;

void thread_idle_read_CAN(void *argument) {
	for (;;) {
		if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

			// Check for Queued CAN Packets on CAN1
			while (osMessageQueueGetCount(CC_CAN_State->CAN1Queue) >= 1) {
				CC_CAN_Generic_t msg;
				if (osMessageQueueGet(CC_CAN_State->CAN1Queue, &msg, 0U, 0U) == osOK) {
					if (msg.header.IDE == CAN_ID_STD) {
						/* Inverter Heartbeat */
						if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								CC_Heartbeat_State->inverterTicks = HAL_GetTick();

								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
					}
				}
			}

			/* Check for Queued CAN Packets on CAN2 */
			while (osMessageQueueGetCount(CC_CAN_State->CAN2Queue) >= 1) {
				CC_CAN_Generic_t msg;
				if (osMessageQueueGet(CC_CAN_State->CAN2Queue, &msg, 0U, 0U) == osOK) {
					/* Packet Handler */
					/* AMS Heartbeat */
					if (msg.header.IDE == CAN_ID_EXT) {
						if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								bool initialised;
								bool HVAn;
								bool HVBn;
								bool precharge;
								bool HVAp;
								bool HVBp;
								uint16_t averageVoltage;
								uint16_t runtime;
								Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn, &precharge, &HVAp,
										&HVBp, &averageVoltage, &runtime);
								CC_Heartbeat_State->amsTicks = HAL_GetTick();
								CC_Global_State->AMS_initialized = initialised;
								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
						/* Shutdown 1 Heartbeat */
						else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0)) {
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								uint8_t segmentState;
								Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
								CC_Heartbeat_State->shutdownOneTicks = HAL_GetTick();
								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
						/* Shutdown 2 Heartbeat */
						else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x02, 0x0)) {
							//CC_LogInfo("SHDN2\r\n", strlen("SHDN2\r\n"));
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								uint8_t segmentState;
								Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
								CC_Heartbeat_State->shutdownTwoTicks = HAL_GetTick();
								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
						/* Shutdown 3 Heartbeat */
						else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x03, 0x0)) {
							//CC_LogInfo("SHDN3\r\n", strlen("SHDN3\r\n"));
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								uint8_t segmentState;
								Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
								CC_Heartbeat_State->shutdownThreeTicks = HAL_GetTick();
								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
						/* Shutdown IMD Heartbeat */
						else if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
							if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								uint8_t pwmState;
								Parse_SHDN_IMD_HeartbeatResponse(*((SHDN_IMD_HeartbeatResponse_t*) &(msg.data)),
										&pwmState);
								CC_Heartbeat_State->shutdownImdTicks = HAL_GetTick();
								osSemaphoreRelease(CC_Heartbeat_State->sem);
							}
						}
						/* Shutdown Triggered Fault */
						else if (msg.header.ExtId == Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0)) {
							if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								CC_Global_State->shutdown_fault = true;
								osSemaphoreRelease(CC_Global_State->sem);
							}
						}
					}

				}
			}
		}

		// update every 20ms
		osDelay(20);
	}

	osThreadTerminate(NULL);
}

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s" };

void state_idle_enter(fsm_t *fsm) {
	/*if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

		// set LV PDM channels
		if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {*/

			CC_Global_State->pdm_channel_states = LV_STARTUP | PDM_POWER_CC_MASK;

			PDM_SetChannelStates_t update_pdm_msg = Compose_PDM_SetChannelStates(CC_Global_State->pdm_channel_states);
			CAN_TxHeaderTypeDef CAN_header = { .ExtId = update_pdm_msg.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA,
					.DLC = sizeof(update_pdm_msg.data), .TransmitGlobalTime = DISABLE, };

			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, update_pdm_msg.data, &CC_CAN_State->CAN2_TxMailbox);

			/*osSemaphoreRelease(CC_CAN_State->sem);
		}*/

		// initialize CC
		CC_Global_State->CC_initialized = true;

		/*osSemaphoreRelease(CC_Global_State->sem);
	}*/

	// start idle specific threads

	// start rtos threads for driving state
	CC_idle_threads = malloc(sizeof(CC_idle_threads_t));

	// driving_update_pdm
	osThreadAttr_t thread_attributes = { 0 };
	thread_attributes.name = "idle_read_CAN";
	thread_attributes.priority = (osPriority_t) osPriorityNormal;
	thread_attributes.stack_size = 1024;

	CC_idle_threads->idle_read_CAN_handle = osThreadNew(thread_idle_read_CAN, NULL, &thread_attributes);

	return;
}

void state_idle_iterate(fsm_t *fsm) {
	int len = 0;
	char x[80];
	// check heartbeats - covered in thread

	// read CAN - covered in thread

	/*
	 // don't let continue
	 if (CC_GlobalState->shutdown_fault) {
	 return;
	 }
	 */

	bool brake_rtd_activated = true;
/*
	if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

		// brake > 20%
		brake_rtd_activated = CC_Tractive_State->brake_value > 200;

		osSemaphoreRelease(CC_Tractive_State->sem);
	}
*/
	if (brake_rtd_activated && (CC_Global_State->AMS_initialized || CC_Global_State->AMS_Debug)
			&& CC_Global_State->CC_initialized) {
		// Illuminate RTD Button
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);

		//if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			// if button is pressed
			if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin)) {
				if (CC_Global_State->rtd_ticks == 0) {
					CC_Global_State->rtd_ticks = HAL_GetTick();
				}

				if ((HAL_GetTick() - CC_Global_State->rtd_ticks) > RTD_ENTER_TICK_COUNT) {
					// rtd button held down for 5 seconds

					//osSemaphoreRelease(CC_Global_State->sem);

					len = sprintf(x, "rtd\r\n");
					CC_LogInfo(x, len);

					// reset timer
					CC_Global_State->rtd_ticks = 0;


					// Enter Driving State
					fsm_changeState(fsm, &drivingState, "RTD Engaged");

					//return;
				}
			}
		/*}

		osSemaphoreRelease(CC_Global_State->sem);*/
	} else {
		// turn off RTD button
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
	}
}

void state_idle_exit(fsm_t *fsm) {

	//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		// send RTD on all CAN lines
		CC_ReadyToDrive_t readyToDrive = Compose_CC_ReadyToDrive();
		CAN_TxHeaderTypeDef CAN_header = { .ExtId = readyToDrive.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1,
				.TransmitGlobalTime = DISABLE, };
		uint8_t data[1] = { 0xF };
		HAL_CAN_AddTxMessage(&hcan1, &CAN_header, data, &CC_CAN_State->CAN1_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan2, &CAN_header, data, &CC_CAN_State->CAN2_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan3, &CAN_header, data, &CC_CAN_State->CAN3_TxMailbox);

		// turn RTD siren on and enable HV power
		//if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			CC_Global_State->pdm_channel_states = CC_Global_State->pdm_channel_states | PDMFLAG_RTD_SIREN | HV_STARTUP;

			PDM_SetChannelStates_t update_pdm_msg = Compose_PDM_SetChannelStates(CC_Global_State->pdm_channel_states);
			CAN_header.ExtId = update_pdm_msg.id;
			CAN_header.DLC = sizeof(update_pdm_msg.data);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, update_pdm_msg.data, &CC_CAN_State->CAN2_TxMailbox);
/*
			osSemaphoreRelease(CC_Global_State->sem);

		}

		osSemaphoreRelease(CC_CAN_State->sem);
	}*/

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

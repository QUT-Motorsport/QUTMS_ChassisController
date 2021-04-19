/**
 ******************************************************************************
 * @file CC_FSM_States.c
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#include "CC_FSM_States.h"
#include "main.h"



#define CAN_1 hcan1
#define CAN_2 hcan2
#define CAN_3 hcan3

#define FAN_CMD_TICK_COUNT 400





state_t deadState = { &state_dead_enter, &state_dead_iterate, &state_dead_exit,
		"Dead_s" };

void state_dead_enter(fsm_t *fsm) {
	return;
}

void state_dead_iterate(fsm_t *fsm) {
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm) {
	return;
}

CC_GlobalState_t *CC_GlobalState = NULL;

state_t startState = { &state_start_enter, &state_start_iterate,
		&state_start_exit, "Start_s" };

void state_start_enter(fsm_t *fsm) {
	if (CC_GlobalState == NULL) {
		/* Assign memory and nullify Global State */
		CC_GlobalState = malloc(sizeof(CC_GlobalState_t));
		memset(CC_GlobalState, 0, sizeof(CC_GlobalState_t));

		/* As CC_GlobalState is accessible across threads
		 * we need to use a semaphore to access and lock it
		 */
		CC_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);

		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT)
				== osOK) {
			/* Bind and configure initial global states */

			/* Skip RTD Sequencing Requiring Brake Pressure */
			CC_GlobalState->RTD_Debug = true;

			/* Ignore ADC Errors */
			CC_GlobalState->ADC_Debug = true;

			/* Boards w/ Heartbeats */
			CC_GlobalState->PDM_Debug = true;
			CC_GlobalState->AMS_Debug = false;
			CC_GlobalState->SHDN_1_Debug = false;
			CC_GlobalState->SHDN_2_Debug = true;
			CC_GlobalState->SHDN_3_Debug = true;
			CC_GlobalState->SHDN_IMD_Debug = true;

			/* Inverters */
			CC_GlobalState->Inverter_Debug = true;

			/* Fans */

			/* Pedal Input Scale */
			CC_GlobalState->pedalScale = 0.3;

			/* Bound state for system operations */
			CC_GlobalState->tractiveActive = false;
			CC_GlobalState->pdmTrackState = LV_STARTUP | CC_MASK;

			CC_GlobalState->finalRtdTicks = 0;
			CC_GlobalState->shutdown_fault = false;

			/* Allocate CAN Queues */
			CC_GlobalState->CAN1Queue = osMessageQueueNew(CC_CAN_QUEUESIZE,
					sizeof(CAN_MSG_Generic_t), NULL);
			CC_GlobalState->CAN2Queue = osMessageQueueNew(CC_CAN_QUEUESIZE,
					sizeof(CAN_MSG_Generic_t), NULL);
			CC_GlobalState->CAN3Queue = osMessageQueueNew(CC_CAN_QUEUESIZE,
					sizeof(CAN_MSG_Generic_t), NULL);
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		/* Ensure CANQueue exists */
		if (CC_GlobalState->CAN1Queue == NULL
				|| CC_GlobalState->CAN2Queue == NULL
				|| CC_GlobalState->CAN3Queue == NULL) {
			Error_Handler();
		}
	}

	/* Set initial pin states */
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
			GPIO_PIN_RESET);

	/* Initiate Startup on PDM */
	PDM_InitiateStartup_t initiateStartup = Compose_PDM_InitiateStartup();
	CAN_TxHeaderTypeDef header = { .ExtId = initiateStartup.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };
	CC_send_can_msg(&CAN_2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
	return;

	/* Debug Tracing */
	//CC_LogInfo("Enter Start\r\n", strlen("Enter Start\r\n"));
	return;
}

void state_start_iterate(fsm_t *fsm) {
	/* Skip boot if PDM Debugging Enabled */
	bool boot = CC_GlobalState->PDM_Debug;
	uint32_t getPowerChannels = 0;

	/* Monitor CAN Queue */
	while (osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1) {
		CAN_MSG_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U)
				== osOK) {
			/* If Startup Ok */
			if (msg.header.ExtId
					== Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0,
							CAN_TYPE_TRANSMIT, 0x00, 0x0)) {
				/* Get Power Channel Values at Boot */
				getPowerChannels = 0;
				Parse_PDM_StartupOk(msg.data, &getPowerChannels);

				/* Initialise Boot */
				boot = true;
			}
		}
	}

	if (boot) {
		/* Set Power Channel Values to Enable on Start */
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(
				CC_GlobalState->pdmTrackState & (~HV_STARTUP));
		CAN_TxHeaderTypeDef header =
				{ .ExtId = pdmStartup.id, .IDE = CAN_ID_EXT,
						.RTR = CAN_RTR_DATA, .DLC = sizeof(pdmStartup.data),
						.TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &header, pdmStartup.data,
				&CC_GlobalState->CAN2_TxMailbox);

		/* Set Heartbeat Timers */
		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT)
				== osOK) {
			CC_GlobalState->startupTicks = HAL_GetTick();
			CC_GlobalState->amsTicks = HAL_GetTick();
			CC_GlobalState->shutdownOneTicks = HAL_GetTick();
			CC_GlobalState->shutdownTwoTicks = HAL_GetTick();
			CC_GlobalState->shutdownThreeTicks = HAL_GetTick();
			CC_GlobalState->shutdownImdTicks = HAL_GetTick();
			CC_GlobalState->inverterTicks = HAL_GetTick();
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		/* Engage Idle State (Waiting for RTD) */
		//fsm_changeState(fsm, &drivingState, "skip to rtd lol");
		fsm_changeState(fsm, &idleState, "PDM Boot Sequence Initiated");
	}
	return;
}

void state_start_exit(fsm_t *fsm) {
	CAN_TxHeaderTypeDef CAN_header;
	CAN_header.IDE = CAN_ID_EXT;
	CAN_header.RTR = CAN_RTR_DATA;
	CAN_header.TransmitGlobalTime = DISABLE;
	/* All CAN Wake or
	 * Confirmation to Idle
	 * Messages go here over CAN */
	//CC_LogInfo("Exit Start\r\n", strlen("Exit Start\r\n"));
	// tell pdm amu fan duty cycle
	int fan_duty_cycle = 100;
	// tell pdm left fan duty cycle
	PDM_SetDutyCycle_t pdm_set_duty_msg = Compose_PDM_SetDutyCycle(
	PDM_PWM_ACU_FAN, fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

	return;
}

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit,
		"Idle_s" };

void state_idle_enter(fsm_t *fsm) {
	/* Calculate Brake Threshold for RTD */
	uint32_t brake_threshold_range = BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN;
	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* Assign Threshold to 20% of Brake Pressure */
		CC_GlobalState->brakePressureThreshold = BRAKE_PRESSURE_MIN
				+ (0.2 * brake_threshold_range);

		/* Init Chassis Controller On */
		CC_GlobalState->ccInit = true;

		/* Torque Commands Default to 0 */
		CC_GlobalState->accelTravel = 0;
		CC_GlobalState->brakeTravel = 0;

		CC_GlobalState->precharge_enabled = false;
		CC_GlobalState->precharge_done = false;

		CC_GlobalState->flashlight_ticks = 0;

		osSemaphoreRelease(CC_GlobalState->sem);
	}

	return;
}

void state_idle_iterate(fsm_t *fsm) {
#ifdef RTD_DEBUG
	fsm_changeState(fsm, &drivingState, "RTD Engaged");
#endif
	char buf[80];
	int len;
	/* Check for Heartbeat Expiry */
	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->amsTicks) > 100
				&& !CC_GlobalState->AMS_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 1 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100
				&& !CC_GlobalState->SHDN_1_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 2 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownTwoTicks) > 100
				&& !CC_GlobalState->SHDN_2_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN2\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown 3 Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownThreeTicks) > 100
				&& !CC_GlobalState->SHDN_3_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN3\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100
				&& !CC_GlobalState->SHDN_IMD_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100
				&& !CC_GlobalState->Inverter_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets on CAN1 */
	while (osMessageQueueGetCount(CC_GlobalState->CAN1Queue) >= 1) {
		CAN_MSG_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN1Queue, &msg, 0U, 0U)
				== osOK) {
			if (msg.header.IDE == CAN_ID_STD) {
				/* inverters don't send anything back rn
				// Inverter Heartbeat
				if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
				*/
			}
		}
	}

	/* Check for Queued CAN Packets on CAN2 */
	while (osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1) {
		CAN_MSG_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U)
				== osOK) {

			/* Packet Handler */
			/* AMS Heartbeat */
			if (msg.header.IDE == CAN_ID_EXT) {
				if (msg.header.ExtId
						== Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
					if (osSemaphoreAcquire(CC_GlobalState->sem,
					SEM_ACQUIRE_TIMEOUT) == osOK) {
						bool initialised = false;
						bool HVAn;
						bool HVBn;
						bool precharge;
						bool HVAp;
						bool HVBp;
						uint16_t averageVoltage;
						uint16_t runtime;
						Parse_AMS_HeartbeatResponse(msg.data, &initialised,
								&HVAn, &HVBn, &precharge, &HVAp, &HVBp,
								&averageVoltage, &runtime);
						if (!CC_GlobalState->amsInit && initialised) {
							len = sprintf(buf, "ams ready\r\n");
							CC_LogInfo(buf, len);
						}
						CC_GlobalState->amsTicks = HAL_GetTick();
						CC_GlobalState->amsInit = initialised;
						osSemaphoreRelease(CC_GlobalState->sem);

					}
				}
				/* Shutdown 1 Heartbeat */
				else if (msg.header.ExtId
						== Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0)) {
					//CC_LogInfo("SHDN1\r\n", strlen("SHDN1\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem,
					SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(
								*((SHDN_HeartbeatResponse_t*) &(msg.data)),
								&segmentState);
						CC_GlobalState->shutdownOneTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if (msg.header.ExtId
						== Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x02, 0x0)) {
					//CC_LogInfo("SHDN2\r\n", strlen("SHDN2\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem,
					SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(
								*((SHDN_HeartbeatResponse_t*) &(msg.data)),
								&segmentState);
						CC_GlobalState->shutdownTwoTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if (msg.header.ExtId
						== Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x03, 0x0)) {
					//CC_LogInfo("SHDN3\r\n", strlen("SHDN3\r\n"));
					if (osSemaphoreAcquire(CC_GlobalState->sem,
					SEM_ACQUIRE_TIMEOUT) == osOK) {
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(
								*((SHDN_HeartbeatResponse_t*) &(msg.data)),
								&segmentState);
						CC_GlobalState->shutdownThreeTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown IMD Heartbeat */
				else if (msg.header.ExtId
						== Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
					uint8_t pwmState;
					Parse_SHDN_IMD_HeartbeatResponse(
							*((SHDN_IMD_HeartbeatResponse_t*) &(msg.data)),
							&pwmState);
					CC_GlobalState->shutdownImdTicks = HAL_GetTick();
				}
				/* Shutdown Triggered Fault */
				else if (msg.header.ExtId
						== Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0)) {
					// TODO DEAL WITH INVERTERS HERE WITH SOFT INVERTER SHUTDOWN
					//					CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault (idle)\r\n", true,
					//							&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					//							&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					//							INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
					CC_GlobalState->shutdown_fault = true;
					len = sprintf(buf, "shutdown triggered \r\n");
					CC_LogInfo(buf, len);
				} else if (msg.header.ExtId == AMS_Ready_ID) {
					if (!CC_GlobalState->precharge_done) {
						len = sprintf(buf, "ams ready recieved \r\n");
						CC_LogInfo(buf, len);
						CC_GlobalState->precharge_done = true;
					}
				}
			}

		}
	}
	/*
	 len = sprintf(buf, "start %d done %d cc init %d ams init %d\r\n",
	 CC_GlobalState->precharge_enabled, CC_GlobalState->precharge_done,
	 CC_GlobalState->ccInit, CC_GlobalState->amsInit);
	 CC_LogInfo(buf, len);*/

	// no trigger if shutdown
	if (CC_GlobalState->ccInit
			&& (CC_GlobalState->amsInit || CC_GlobalState->AMS_Debug)
			/*&& !CC_GlobalState->shutdown_fault*/) {
		if (!CC_GlobalState->precharge_enabled) {
			if (HAL_GetTick() - CC_GlobalState->flashlight_ticks > 500) {
				HAL_GPIO_TogglePin(HSOUT_RTD_LED_GPIO_Port,
				HSOUT_RTD_LED_Pin);
				CC_GlobalState->flashlight_ticks = HAL_GetTick();
			}

			if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin)) {

				if (CC_GlobalState->precharge_ticks == 0) {
					len = sprintf(buf, "precharge btn press\r\n");
					CC_LogInfo(buf, len);
					// button just got pressed so init counter
					CC_GlobalState->precharge_ticks = HAL_GetTick();
				}
				if ((HAL_GetTick() - CC_GlobalState->precharge_ticks) > 200) {
					len = sprintf(buf, "precharge started\r\n");
					CC_LogInfo(buf, len);
					// button held for 2 seconds
					CC_GlobalState->precharge_enabled = true;
					CC_GlobalState->precharge_done = false;

					CC_GlobalState->pdmTrackState =
							CC_GlobalState->pdmTrackState | HV_STARTUP;
					PDM_SetChannelStates_t rtdSiren =
							Compose_PDM_SetChannelStates(
									CC_GlobalState->pdmTrackState);
					CAN_TxHeaderTypeDef sirenHeader = { .ExtId = rtdSiren.id,
							.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
									sizeof(rtdSiren.data), .TransmitGlobalTime =
									DISABLE, };
					CC_send_can_msg(&hcan2, &sirenHeader, rtdSiren.data,
							&CC_GlobalState->CAN2_TxMailbox);

					// send CAN msg to AMS to start precharge
					AMS_StartUp_t ams_startup = Compose_AMS_StartUp();
					CAN_TxHeaderTypeDef header = { .ExtId = ams_startup.id,
							.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0,
							.TransmitGlobalTime = DISABLE, };
					CC_send_can_msg(&hcan2, &header, NULL,
							&CC_GlobalState->CAN2_TxMailbox);

				}
			} else {
				// button is not pressed so reset counter
				CC_GlobalState->precharge_ticks = 0;
			}
		} else {
			if (!CC_GlobalState->precharge_done) {
				// waiting for AMS to finish precharge
				HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port,
				HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
				len = sprintf(buf, "waiting for precharge\r\n");
				CC_LogInfo(buf, len);
			}
			if (CC_GlobalState->precharge_done) {
				// AMS has said precharge is done
				len = sprintf(buf, "precharge done\r\n");
				CC_LogInfo(buf, len);

				uint16_t brake_threshold = BRAKE_PRESSURE_MIN
						+ (0.3 * (BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN));

				// If Brake Pressure > 30%
				uint16_t raw;
				if (CC_GlobalState->RTD_Debug) {
					raw = brake_threshold;
				} else {
					HAL_ADC_Start(&hadc3);
					raw = HAL_ADC_GetValue(&hadc3);
				}

				// wait for pressure to be >30 and button to be pressed for 5 seconds
				if (raw >= brake_threshold) {
					// Illuminate RTD Button
					HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port,
					HSOUT_RTD_LED_Pin, GPIO_PIN_SET);

					// If RTD Button Engaged
					if (osSemaphoreAcquire(CC_GlobalState->sem,
					SEM_ACQUIRE_TIMEOUT) == osOK) {
						if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port,
						RTD_INPUT_Pin)) {
							if (CC_GlobalState->finalRtdTicks == 0) {
								CC_GlobalState->finalRtdTicks = HAL_GetTick();
							}

							if ((HAL_GetTick() - CC_GlobalState->finalRtdTicks)
									>= 5000) {
								// Enter Driving State
								fsm_changeState(fsm, &drivingState,
										"RTD Engaged");
							}
						}
						osSemaphoreRelease(CC_GlobalState->sem);
					}

				} else {
					HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port,
					HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
				}
			}
		}
	}
}

void state_idle_exit(fsm_t *fsm) {
	char buf[80];
	int len;
	/* Broadcast RTD on all CAN lines */
	CC_ReadyToDrive_t readyToDrive = Compose_CC_ReadyToDrive();
	CAN_TxHeaderTypeDef header = { .ExtId = readyToDrive.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };
	CC_send_can_msg(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
	CC_send_can_msg(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
	CC_send_can_msg(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);

	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		CC_GlobalState->readyToDriveTicks = HAL_GetTick();
		osSemaphoreRelease(CC_GlobalState->sem);
	}
#ifndef RTD_DEBUG
	// RTD Siren & HV Startup
	CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
			| RTD_SIREN_MASK | HV_STARTUP;
	PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(
			CC_GlobalState->pdmTrackState);
	CAN_TxHeaderTypeDef sirenHeader = { .ExtId = rtdSiren.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(rtdSiren.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&hcan2, &sirenHeader, rtdSiren.data,
			&CC_GlobalState->CAN2_TxMailbox);
#endif
	len = sprintf(buf, "exit idle\r\n");
	CC_LogInfo(buf, len);

	return;
}

state_t debugState = { &state_debug_enter, &state_debug_iterate,
		&state_debug_exit, "Debug_s" };

void state_debug_enter(fsm_t *fsm) {
	PDM_InitiateStartup_t init = Compose_PDM_InitiateStartup();
	uint8_t test[1] = { 0 };
	CAN_TxHeaderTypeDef header = { .ExtId = init.id, .IDE = CAN_ID_EXT, .RTR =
	CAN_RTR_DATA, .DLC = sizeof(test), .TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&hcan2, &header, test, &CC_GlobalState->CAN2_TxMailbox);
	osDelay(100);
	return;
}

void state_debug_iterate(fsm_t *fsm) {
	for (int i = 1; i <= 32; i++) {
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(
				1 << i);
		CAN_TxHeaderTypeDef header = { .ExtId = pdmStartup.id, .IDE =
		CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(pdmStartup.data),
				.TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &header, pdmStartup.data,
				&CC_GlobalState->CAN2_TxMailbox);
		osDelay(100);
	}

	return;
}

void state_debug_exit(fsm_t *fsm) {
//CC_LogInfo("Exit Debugging\r\n", strlen("Exit Debugging\r\n"));
	return;
}

/**
 ******************************************************************************
 * @file CC_FSM_States.c
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#include "CC_FSM_States.h"
#include "main.h"

#define NUM_BRAKE_SENSORS 2
#define NUM_ACCEL_SENSORS 3

#define POT_DESYNC 250

#define CAN_1 hcan1
#define CAN_2 hcan2
#define CAN_3 hcan3

#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

#define CC_MASK PDM_POWER_CC_MASK
#define RTD_SIREN_MASK PDMFLAG_RTD_SIREN

#define INVERTER_CMD_TICK_COUNT 10
#define INVERTER_ENABLE_TICK_COUNT 15

#define NUM_INVERTERS 2
#define INVERTER_VAR_ACCEL 0x01
#define INVERTER_VAR_BRAKE 0x02

#define FAN_CMD_TICK_COUNT 400

#define BRAKE_UPDATE_TICK_COUNT 20

uint16_t inverter_node_ids[NUM_INVERTERS] = { INVERTER_LEFT_NODE_ID,
INVERTER_RIGHT_NODE_ID };

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
			CC_GlobalState->RTD_Debug = false;

			/* Ignore ADC Errors */
			CC_GlobalState->ADC_Debug = true;

			/* Boards w/ Heartbeats */
			CC_GlobalState->PDM_Debug = false;
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
				/* Inverter Heartbeat */
				if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
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

	/* RTD Siren & HV Startup */
	CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
			| RTD_SIREN_MASK | HV_STARTUP;
	PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(
			CC_GlobalState->pdmTrackState);
	CAN_TxHeaderTypeDef sirenHeader = { .ExtId = rtdSiren.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(rtdSiren.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&hcan2, &sirenHeader, rtdSiren.data,
			&CC_GlobalState->CAN2_TxMailbox);

	len = sprintf(buf, "exit idle\r\n");
	CC_LogInfo(buf, len);

	return;
}

state_t drivingState = { &state_driving_enter, &state_driving_iterate,
		&state_driving_exit, "Driving_s" };

void state_driving_enter(fsm_t *fsm) {
	char buf[80];
	int len;

	len = sprintf(buf, "enter rtd\r\n");
	CC_LogInfo(buf, len);

	CAN_TxHeaderTypeDef CAN_header;
	CAN_header.IDE = CAN_ID_EXT;
	CAN_header.RTR = CAN_RTR_DATA;
	CAN_header.TransmitGlobalTime = DISABLE;

	/* If AMS Contactors Closed & BMS' Healthy */

	/* Enable all channels on PDM */
// TODO Fix Bitwise Flip on enter IDLE State under current PDM Startup Sequence
	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		CC_GlobalState->tractiveActive = true;
		CC_GlobalState->faultDetected = false;
		CC_GlobalState->rtdLightActive = true;

		memset(CC_GlobalState->rollingBrakeValues, 0, 2 * sizeof(uint32_t));
		memset(CC_GlobalState->rollingAccelValues, 0, 3 * sizeof(uint32_t));

		CC_GlobalState->brakeMin[0] = BRAKE_PEDAL_ONE_MIN;
		CC_GlobalState->brakeMin[1] = BRAKE_PEDAL_TWO_MIN;
		CC_GlobalState->brakeMax[0] = BRAKE_PEDAL_ONE_MAX;
		CC_GlobalState->brakeMax[1] = BRAKE_PEDAL_TWO_MAX;

		CC_GlobalState->accelMin[0] = ACCEL_PEDAL_ONE_MIN;
		CC_GlobalState->accelMax[0] = ACCEL_PEDAL_ONE_MAX;
		CC_GlobalState->accelMin[1] = ACCEL_PEDAL_TWO_MIN;
		CC_GlobalState->accelMax[1] = ACCEL_PEDAL_TWO_MAX;
		CC_GlobalState->accelMin[2] = ACCEL_PEDAL_THREE_MIN;
		CC_GlobalState->accelMax[2] = ACCEL_PEDAL_THREE_MAX;

		// set duty cycle of left and right, and amu fans
		uint16_t fan_duty_cycle = 100;

		// tell pdm left fan duty cycle
		PDM_SetDutyCycle_t pdm_set_duty_msg = Compose_PDM_SetDutyCycle(
		PDM_PWM_LEFT_FAN, fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);

		// tell pdm right fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN,
				fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);

		// tell pdm amu fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN,
				fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);

		osSemaphoreRelease(CC_GlobalState->sem);
	}
	/* Start Polling ADC */
	HAL_ADC_Start_DMA(&hadc2, CC_GlobalState->brakeAdcValues, 2);
	HAL_ADC_Start_DMA(&hadc1, CC_GlobalState->accelAdcValues, 3);

	/* Run MicroBasic Script on Inverter */
	CC_SetBool_t runLeftScript = Compose_CC_SetBool(INVERTER_LEFT_NODE_ID, 0x01,
			0xFFFFFFFF);
	CAN_TxHeaderTypeDef runLeftHeader = { .StdId = runLeftScript.id, .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = sizeof(runLeftScript.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&CAN_1, &runLeftHeader, runLeftScript.data,
			&CC_GlobalState->CAN1_TxMailbox);

	CC_SetBool_t runRightScript = Compose_CC_SetBool(INVERTER_RIGHT_NODE_ID,
			0x01, 0xFFFFFFFF);
	CAN_TxHeaderTypeDef runRightHeader = { .StdId = runRightScript.id, .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = sizeof(runRightScript.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&CAN_1, &runRightHeader, runRightScript.data,
			&CC_GlobalState->CAN1_TxMailbox);

	len = sprintf(buf, "exit rtd\r\n");
	CC_LogInfo(buf, len);
	return;
}

void state_driving_iterate(fsm_t *fsm) {

	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* Flash RTD */
		if ((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) > 1000) {
			HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
					!CC_GlobalState->rtdLightActive);
			CC_GlobalState->rtdLightActive = !CC_GlobalState->rtdLightActive;
			CC_GlobalState->readyToDriveTicks = HAL_GetTick();

			CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
					& (~RTD_SIREN_MASK);
			PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(
					CC_GlobalState->pdmTrackState);
			CAN_TxHeaderTypeDef sirenHeader = { .ExtId = rtdSiren.id, .IDE =
			CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(rtdSiren.data),
					.TransmitGlobalTime = DISABLE, };
			CC_send_can_msg(&hcan2, &sirenHeader, rtdSiren.data,
					&CC_GlobalState->CAN2_TxMailbox);
		}
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->amsTicks) > 100
				&& !CC_GlobalState->AMS_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&CAN_1, &CAN_2, &CAN_3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100
				&& !CC_GlobalState->SHDN_1_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
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
		char x[80];
		int len;
		CAN_MSG_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN1Queue, &msg, 0U, 0U)
				== osOK) {
			if (msg.header.IDE == CAN_ID_STD) {
				/* Inverter Heartbeat */
				if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {
					CC_GlobalState->inverterTicks = HAL_GetTick();
					len = sprintf(x, "INVERTER HEARTBEAT\r\n");
					CC_LogInfo(x, len);
				}
				/* Inverter Response Packet */
				else if (msg.header.StdId == 0x580 + INVERTER_LEFT_NODE_ID) {
					/* Motor RPM Response Packet */
					if ((msg.data[2] << 8 | msg.data[1]) == 0x210A) {
						/* Parse Motor RPM */
						int16_t motorRPM = 0;
						Parse_CC_RequestRPM(msg.data, &motorRPM);

						/* Echo Motor RPM */
						len = sprintf(x, "[%li] Got RPM from CAN1: %i\r\n",
								(HAL_GetTick() - CC_GlobalState->startupTicks)
										/ 1000, motorRPM);
						//CC_LogInfo(x, len);
					} else {
						/* Echo CAN Packet if index not recognised */
						len = sprintf(x,
								"[%li] Got CAN msg from CAN1: %02lX\r\n",
								(HAL_GetTick() - CC_GlobalState->startupTicks)
										/ 1000, msg.header.StdId);
						//CC_LogInfo(x, len);
					}
				}
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
			if (msg.header.ExtId
					== Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
				if (osSemaphoreAcquire(CC_GlobalState->sem,
				SEM_ACQUIRE_TIMEOUT) == osOK) {
					bool initialised;
					bool HVAn;
					bool HVBn;
					bool precharge;
					bool HVAp;
					bool HVBp;
					uint16_t averageVoltage;
					uint16_t runtime;
					Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn,
							&HVBn, &precharge, &HVAp, &HVBp, &averageVoltage,
							&runtime);
					CC_GlobalState->amsTicks = HAL_GetTick();
					CC_GlobalState->amsInit = initialised;
					osSemaphoreRelease(CC_GlobalState->sem);
				}
			}
			/* Shutdown 1 Heartbeat */
			else if (msg.header.ExtId
					== Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0)) {
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
			/* Shutdown IMD Heartbeat */
			else if (msg.header.ExtId
					== Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
				if (osSemaphoreAcquire(CC_GlobalState->sem,
				SEM_ACQUIRE_TIMEOUT) == osOK) {
					uint8_t pwmState;
					Parse_SHDN_IMD_HeartbeatResponse(
							*((SHDN_IMD_HeartbeatResponse_t*) &(msg.data)),
							&pwmState);
					CC_GlobalState->shutdownImdTicks = HAL_GetTick();
					osSemaphoreRelease(CC_GlobalState->sem);
				}
			}
			/* Shutdown Triggered Fault */
			else if (msg.header.ExtId
					== Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0)) {
				CC_GlobalState->ccInit = Send_CC_FatalShutdown(
						"Fatal Shutdown Trigger Fault\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox,
						&CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3,
						&huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
				fsm_changeState(fsm, &idleState, "Resetting to Idle to Clean");
			}
		}
	}

	/*
	 * Read 3 Throttle ADC Values
	 * Read 2 Brake ADC Values
	 */
	uint32_t brake_travel[2];
	uint32_t accel_travel[3];
	uint32_t brake_sum = 0;
	uint32_t accel_sum = 0;
	uint32_t pedal_bounds = MAX_DUTY_CYCLE * CC_GlobalState->pedalScale;
	char x[80];
	uint32_t len;

	/* Echo ADC Failure for Debugging */
	if (CC_GlobalState->faultDetected) {
		//CC_LogInfo("ADC Fault Detected\r\n", strlen("ADC Fault Detected\r\n"));
	}
	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* Calculatee Brake Positions with Filter */
		for (int i = 0; i < NUM_BRAKE_SENSORS; i++) {
			/* Fetch Value & Apply Filter */
			uint32_t y = CC_GlobalState->pedalScale
					* CC_GlobalState->brakeAdcValues[i]
					+ (1.0 - CC_GlobalState->pedalScale)
							* CC_GlobalState->rollingBrakeValues[i];
			CC_GlobalState->rollingBrakeValues[i] = y;

			/* Map to Max Duty Cycle */
			brake_travel[i] = map(CC_GlobalState->rollingBrakeValues[i],
					CC_GlobalState->brakeMin[i], CC_GlobalState->brakeMax[i], 0,
					MAX_DUTY_CYCLE);
		}

		/* Calculate Accel Positions with Filter */
		for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
			/* Fetch Value & Apply Filter */
			uint32_t y = CC_GlobalState->pedalScale
					* CC_GlobalState->accelAdcValues[i]
					+ (1.0 - CC_GlobalState->pedalScale)
							* CC_GlobalState->rollingAccelValues[i];
			CC_GlobalState->rollingAccelValues[i] = y;

			/* Map to Max Duty Cycle */
			accel_travel[i] = map(CC_GlobalState->rollingAccelValues[i],
					CC_GlobalState->accelMin[i], CC_GlobalState->accelMax[i], 0,
					MAX_DUTY_CYCLE);
		}

		/* Calculate Faulty ADC Reading */
		bool currentFault = false;
		for (int i = 0; i < NUM_BRAKE_SENSORS; i++) {
			for (int y = 0; y < NUM_BRAKE_SENSORS; y++) {
				if (brake_travel[i] < (int32_t) brake_travel[y] - pedal_bounds
						|| brake_travel[i]
								> (int32_t) brake_travel[y] + pedal_bounds) {
					currentFault = true;
				}
			}
		}
		for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
			for (int y = 0; y < NUM_ACCEL_SENSORS; y++) {
				if (accel_travel[i] < (int32_t) accel_travel[y] - pedal_bounds
						|| accel_travel[i]
								> (int32_t) accel_travel[y] + pedal_bounds) {
					currentFault = true;
				}
			}
		}

		/* Fault Handling - Comment Out to Stop Faulting */
		if (currentFault) {
			/* New Fault */
			if (!CC_GlobalState->faultDetected) {
				//				CC_GlobalState->faultDetected = currentFault;
				//				CC_GlobalState->implausibleTicks = HAL_GetTick();
			}
		} else {
			/* Reset Tripped Fault */
			CC_GlobalState->faultDetected = currentFault;
			CC_GlobalState->implausibleTicks = 0;
		}

		/* Convert Pedal Positions to Torque Commands */
		CC_GlobalState->brakeTravel = MAX_DUTY_CYCLE - brake_travel[0];
		CC_GlobalState->accelTravel = MAX_DUTY_CYCLE - accel_travel[0];

		// apply plausibility check to accelerator for pedal disconnect
		for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
			// apply plausibilty check for pedal disconnect
			if ((CC_GlobalState->accelAdcValues[i]
					> 1.2 * CC_GlobalState->accelMax[i])
					|| (CC_GlobalState->accelAdcValues[i]
							< 1.2 * CC_GlobalState->accelMin[i])) {
				// basically if any pedal value is outside the acceptable range set pedal value to 0
				CC_GlobalState->accelTravel = 0;
			}
		}

		// apply dead zones
		if (CC_GlobalState->accelTravel < DEAD_ZONE_ACCEL) {
			CC_GlobalState->accelTravel = 0;
		}
		if (CC_GlobalState->brakeTravel < DEAD_ZONE_BRAKE) {
			CC_GlobalState->brakeTravel = 0;
		}

		osSemaphoreRelease(CC_GlobalState->sem);
	}

// print pedal positions
	if (false) {
		len = sprintf(x, "Accel: %d Brake: %d\r\n", CC_GlobalState->accelTravel,
				CC_GlobalState->brakeTravel);
		CC_LogInfo(x, len);
	}

	/*
	 * If Throttle and Brake Implausibility State Clock < 100ms
	 * Suspend Tractive System Operations
	 */
	if (CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug
			&& CC_GlobalState->tractiveActive
			&& (HAL_GetTick() - CC_GlobalState->implausibleTicks) >= 100) {
		CC_GlobalState->tractiveActive = false;
		CC_LogInfo("Disabling Tractive Operations\r\n",
				strlen("Disabling Tractive Operations\r\n"));
	}

	/*
	 * Motor Control Commands
	 */

	CAN_TxHeaderTypeDef inverter_header = { 0 };
	inverter_header.IDE = CAN_ID_STD;
	inverter_header.RTR = CAN_RTR_DATA;
	inverter_header.TransmitGlobalTime = DISABLE;

	uint8_t result = 0;

// send enable command
	if (((HAL_GetTick() - CC_GlobalState->inverter_enable_ticks)
			>= INVERTER_ENABLE_TICK_COUNT)) {
		CC_SetBool_t inverter_enable = { 0 };

		for (int i = 0; i < NUM_INVERTERS; i++) {
			inverter_enable = Compose_CC_SetBool(inverter_node_ids[i], 0x01,
					0xFFFFFFFF);
			inverter_header.StdId = inverter_enable.id;
			inverter_header.DLC = 8;

			result = CC_send_can_msg(&CAN_1, &inverter_header,
					inverter_enable.data, &CC_GlobalState->CAN1_TxMailbox);

		}

		CC_GlobalState->inverter_enable_ticks = HAL_GetTick();
	}

// send accel and brake
	if (((HAL_GetTick() - CC_GlobalState->inverter_cmd_ticks)
			>= INVERTER_CMD_TICK_COUNT) && !CC_GlobalState->faultDetected) {
		CC_SetVariable_t inverter_cmd = { 0 };

		for (int i = 0; i < NUM_INVERTERS; i++) {
			len = sprintf(x, "Inverter: %d Command - A: %d B: %d\r\n", i,
					CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
			CC_LogInfo(x, len);

			// accel
			inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i],
					INVERTER_VAR_ACCEL, CC_GlobalState->accelTravel);
			inverter_header.StdId = inverter_cmd.id;
			inverter_header.DLC = 8;
			result = CC_send_can_msg(&CAN_1, &inverter_header,
					inverter_cmd.data, &CC_GlobalState->CAN1_TxMailbox);

			// brake
			inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i],
					INVERTER_VAR_BRAKE, CC_GlobalState->brakeTravel);
			inverter_header.StdId = inverter_cmd.id;
			inverter_header.DLC = 8;
			result = CC_send_can_msg(&CAN_1, &inverter_header,
					inverter_cmd.data, &CC_GlobalState->CAN1_TxMailbox);
		}

		CC_GlobalState->inverter_cmd_ticks = HAL_GetTick();
	}

	/*
	 * If Throttle or Brake Implausibility State Clock > 1000ms
	 * Engage Soft Shutdown (Reset to Idle)
	 */
//	if(CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug && !CC_GlobalState->tractiveActive && (HAL_GetTick() - CC_GlobalState->implausibleTicks) >= 1000)
//	{
//		/* Send Zero Command */
//		CC_SetVariable_t accelZeroLeftCommand = Compose_CC_SetVariable(INVERTER_LEFT_NODE_ID,
//				0x01,
//				0x0000);
//		CAN_TxHeaderTypeDef accelZeroLeftHeader =
//		{
//				.StdId = accelZeroLeftCommand.id,
//				.IDE = CAN_ID_STD,
//				.RTR = CAN_RTR_DATA,
//				.DLC = 8,
//				.TransmitGlobalTime = DISABLE,
//		};
//		CC_send_can_msg(&CAN_1, &accelZeroLeftHeader, accelZeroLeftCommand.data, &CC_GlobalState->CAN1_TxMailbox);
//
//		CC_SetVariable_t accelZeroRightCommand = Compose_CC_SetVariable(INVERTER_RIGHT_NODE_ID,
//				0x01,
//				0x0000);
//		CAN_TxHeaderTypeDef accelZeroRightHeader =
//		{
//				.StdId = accelZeroRightCommand.id,
//				.IDE = CAN_ID_STD,
//				.RTR = CAN_RTR_DATA,
//				.DLC = 8,
//				.TransmitGlobalTime = DISABLE,
//		};
//		CC_send_can_msg(&CAN_1, &accelZeroRightHeader, accelZeroRightCommand.data, &CC_GlobalState->CAN1_TxMailbox);
//
//		/* Broadcast Soft Shutdown on all CAN lines */
//		CC_SoftShutdown_t softShutdown = Compose_CC_SoftShutdown();
//		CAN_TxHeaderTypeDef header =
//		{
//				.ExtId = softShutdown.id,
//				.IDE = CAN_ID_EXT,
//				.RTR = CAN_RTR_DATA,
//				.DLC = 1,
//				.TransmitGlobalTime = DISABLE,
//		};
//		uint8_t data[1] = {0xF};
//		CC_send_can_msg(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
//		CC_send_can_msg(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
//		CC_send_can_msg(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
//		fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");
//	}
	/*
	 * If 500ms has exceeded since SoC Request
	 * Request State of Charge
	 */

	if ((HAL_GetTick() - CC_GlobalState->brakelight_ticks)
			> BRAKE_UPDATE_TICK_COUNT) {
		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT)
				== osOK) {
			if (CC_GlobalState->brakeTravel > BRAKELIGHT_THRESHOLD) {
				CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
						| BRAKE_LIGHT_MASK;

			} else {
				CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
						& (~BRAKE_LIGHT_MASK);
			}
			osSemaphoreRelease(CC_GlobalState->sem);
		}
		PDM_SetChannelStates_t brakeLightState = Compose_PDM_SetChannelStates(
				CC_GlobalState->pdmTrackState);
		CAN_TxHeaderTypeDef brakeHeader = { .ExtId = brakeLightState.id, .IDE =
		CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(brakeLightState.data),
				.TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &brakeHeader, brakeLightState.data,
				&CC_GlobalState->CAN2_TxMailbox);

		CC_GlobalState->brakelight_ticks = HAL_GetTick();
	}
}

void state_driving_exit(fsm_t *fsm) {
	CAN_TxHeaderTypeDef CAN_header;
	CAN_header.IDE = CAN_ID_EXT;
	CAN_header.RTR = CAN_RTR_DATA;
	CAN_header.TransmitGlobalTime = DISABLE;
// set duty cycle of left and right, and amu fans
	uint16_t fan_duty_cycle = 0;

// tell pdm left fan duty cycle
	PDM_SetDutyCycle_t pdm_set_duty_msg = Compose_PDM_SetDutyCycle(
	PDM_PWM_LEFT_FAN, fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

// tell pdm right fan duty cycle
	pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN,
			fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

// tell pdm amu fan duty cycle
	pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN,
			fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&CAN_2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

	/* Broadcast Soft Shutdown */
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

/**
 ******************************************************************************
 * @file CC_FSM_States.c
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#include "CC_FSM_States.h"
#include "main.h"

#define BRAKE_PRESSURE_MIN 400
#define BRAKE_PRESSURE_MAX 1400

#define BRAKE_PEDAL_ONE_MIN 2420
#define BRAKE_PEDAL_ONE_MAX 3200
#define BRAKE_PEDAL_TWO_MIN 2320
#define BRAKE_PEDAL_TWO_MAX 3100

#define ACCEL_PEDAL_ONE_MIN 2650
#define ACCEL_PEDAL_ONE_MAX 3150
#define ACCEL_PEDAL_TWO_MIN 2800
#define ACCEL_PEDAL_TWO_MAX 3300
#define ACCEL_PEDAL_THREE_MIN 2800
#define ACCEL_PEDAL_THREE_MAX 3300

#define POT_DESYNC 250

#define MAX_DUTY_CYCLE 1000

#define CAN_1 hcan1
#define CAN_2 hcan2
#define CAN_3 hcan3

#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

#define CC_MASK PDM_POWER_CC_MASK
#define RTD_SIREN_MASK PDMFLAG_RTD_SIREN

state_t deadState = {&state_dead_enter, &state_dead_iterate, &state_dead_exit, "Dead_s"};

void state_dead_enter(fsm_t *fsm)
{
	return;
}

void state_dead_iterate(fsm_t *fsm)
{
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm)
{
	return;
}

state_t startState = {&state_start_enter, &state_start_iterate, &state_start_exit, "Start_s"};

void state_start_enter(fsm_t *fsm)
{
	if(CC_GlobalState == NULL)
	{
		/* Assign memory and nullify Global State */
		CC_GlobalState = malloc(sizeof(CC_GlobalState_t));
		memset(CC_GlobalState, 0, sizeof(CC_GlobalState_t));

		/* As CC_GlobalState is accessible across threads
		 * we need to use a semaphore to access and lock it
		 */
		CC_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);

		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			/* Bind and configure initial global states */

			/* Skip RTD Sequencing Requiring Brake Pressure */
			CC_GlobalState->RTD_Debug = true;

			/* Ignore ADC Errors */
			CC_GlobalState->ADC_Debug = false;

			/* Boards w/ Heartbeats */
			CC_GlobalState->PDM_Debug = false;
			CC_GlobalState->AMS_Debug = false;
			CC_GlobalState->SHDN_1_Debug = false;
			CC_GlobalState->SHDN_2_Debug = true;
			CC_GlobalState->SHDN_3_Debug = true;
			CC_GlobalState->SHDN_IMD_Debug = true;

			/* Inverters */
			CC_GlobalState->Inverter_Debug = true;

			/* Bound state for system operations */
			CC_GlobalState->tractiveActive = false;
			CC_GlobalState->pdmTrackState = LV_STARTUP | CC_MASK;

			/* Allocate CAN Queues */
			CC_GlobalState->CAN1Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			CC_GlobalState->CAN2Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			CC_GlobalState->CAN3Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		/* Ensure CANQueue exists */
		if(CC_GlobalState->CAN1Queue == NULL || CC_GlobalState->CAN2Queue == NULL || CC_GlobalState->CAN3Queue == NULL)
		{
			Error_Handler();
		}
	}

	/* Set initial pin states */
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);

	/* Initiate Startup on PDM */
	PDM_InitiateStartup_t initiateStartup = Compose_PDM_InitiateStartup();
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = initiateStartup.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};
	uint8_t data[1] = {0xF};
	HAL_CAN_AddTxMessage(&CAN_2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
	return;

	/* Debug Tracing */
	//CC_LogInfo("Enter Start\r\n", strlen("Enter Start\r\n"));
	return;
}

void state_start_iterate(fsm_t *fsm)
{
	/* Skip boot if PDM Debugging Enabled */
	bool boot = CC_GlobalState->PDM_Debug;
	uint32_t getPowerChannels = 0;

	/* Monitor CAN Queue */
	while(osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK)
		{
			/* If Startup Ok */
			if(msg.header.ExtId == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0,
					CAN_TYPE_TRANSMIT, 0x00, 0x0))
			{
				/* Get Power Channel Values at Boot */
				getPowerChannels = 0;
				Parse_PDM_StartupOk(msg.data, &getPowerChannels);

				/* Initialise Boot */
				boot = true;
			}
		}
	}

	if(boot)
	{
		/* Set Power Channel Values to Enable on Start */
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(CC_GlobalState->pdmTrackState & (~HV_STARTUP));
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = pdmStartup.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(pdmStartup.data),
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&hcan2, &header, pdmStartup.data, &CC_GlobalState->CAN2_TxMailbox);

		/* Set Heartbeat Timers */
		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
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
		fsm_changeState(fsm, &idleState, "PDM Boot Sequence Initiated");
	}
	return;
}

void state_start_exit(fsm_t *fsm)
{
	/* All CAN Wake or
	 * Confirmation to Idle
	 * Messages go here over CAN */
	//CC_LogInfo("Exit Start\r\n", strlen("Exit Start\r\n"));
	return;
}

state_t idleState = {&state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s"};

void state_idle_enter(fsm_t *fsm)
{
	/* Calculate Brake Threshold for RTD */
	uint32_t brake_threshold_range = BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN;
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Assign Threshold to 20% of Brake Pressure */
		CC_GlobalState->brakePressureThreshold = BRAKE_PRESSURE_MIN + (0.2 * brake_threshold_range);

		/* Init Chassis Controller On */
		CC_GlobalState->ccInit = true;
		osSemaphoreRelease(CC_GlobalState->sem);
	}
	return;
}

void state_idle_iterate(fsm_t *fsm)
{
	/* Check for Heartbeat Expiry */
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->amsTicks) > 100 && !CC_GlobalState->AMS_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown 1 Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100 && !CC_GlobalState->SHDN_1_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown 2 Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownTwoTicks) > 100 && !CC_GlobalState->SHDN_2_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN2\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown 3 Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownThreeTicks) > 100 && !CC_GlobalState->SHDN_3_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN3\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100 && !CC_GlobalState->Inverter_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets on CAN1 */
	while(osMessageQueueGetCount(CC_GlobalState->CAN1Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN1Queue, &msg, 0U, 0U) == osOK)
		{
			if(msg.header.IDE == CAN_ID_STD) {
				/* Inverter Heartbeat */
				if(msg.header.StdId == 0x700+INVERTER_LEFT_NODE_ID)
				{
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
			}
		}
	}

	/* Check for Queued CAN Packets on CAN2 */
	while(osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK)
		{
			/* Packet Handler */
			/* AMS Heartbeat */
			if(msg.header.IDE == CAN_ID_EXT) {
				if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
				{
					if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						bool initialised = false; bool HVAn; bool HVBn; bool precharge; bool HVAp; bool HVBp; uint16_t averageVoltage; uint16_t runtime;
						Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn, &precharge, &HVAp, &HVBp, &averageVoltage, &runtime);
						CC_GlobalState->amsTicks = HAL_GetTick();
						CC_GlobalState->amsInit = initialised;
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 1 Heartbeat */
				else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0))
				{
					//CC_LogInfo("SHDN1\r\n", strlen("SHDN1\r\n"));
					if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
						CC_GlobalState->shutdownOneTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x02, 0x0))
				{
					//CC_LogInfo("SHDN2\r\n", strlen("SHDN2\r\n"));
					if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
						CC_GlobalState->shutdownTwoTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown 2 Heartbeat */
				else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x03, 0x0))
				{
					//CC_LogInfo("SHDN3\r\n", strlen("SHDN3\r\n"));
					if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
						CC_GlobalState->shutdownThreeTicks = HAL_GetTick();
						osSemaphoreRelease(CC_GlobalState->sem);
					}
				}
				/* Shutdown IMD Heartbeat */
				else if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
				{
					uint8_t pwmState;
					Parse_SHDN_IMD_HeartbeatResponse(*((SHDN_IMD_HeartbeatResponse_t*)&(msg.data)), &pwmState);
					CC_GlobalState->shutdownImdTicks = HAL_GetTick();
				}
				/* Shutdown Triggered Fault */
				else if(msg.header.ExtId == Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0))
				{
					// TODO DEAL WITH INVERTERS HERE WITH SOFT INVERTER SHUTDOWN
					CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault\r\n", true,
							&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
							&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
							INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				}
			}


		}
	}

	/* If Brake Pressure > 20% */
	uint16_t raw;
	if(CC_GlobalState->RTD_Debug)
	{
		int brake_threshold_range = BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN;
		raw = BRAKE_PRESSURE_MIN + (0.3 * brake_threshold_range);
	}
	else
	{
		HAL_ADC_Start(&hadc3);
		raw = HAL_ADC_GetValue(&hadc3);
	}
	if(raw > CC_GlobalState->brakePressureThreshold
			&& (CC_GlobalState->amsInit || CC_GlobalState->AMS_Debug)
			&& CC_GlobalState->ccInit)
	{
		/* Illuminate RTD Button */
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);
		/* If RTD Button Engaged */
		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			if(HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin) && (HAL_GetTick() - CC_GlobalState->finalRtdTicks) >= 5000)
			{
				/* Enter Driving State */
				fsm_changeState(fsm, &drivingState, "RTD Engaged");
			}
			osSemaphoreRelease(CC_GlobalState->sem);
		}
	}
	else
	{
		HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
	}
}

void state_idle_exit(fsm_t *fsm)
{
	/* Broadcast RTD on all CAN lines */
	CC_ReadyToDrive_t readyToDrive = Compose_CC_ReadyToDrive();
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = readyToDrive.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};
	uint8_t data[1] = {0xF};
	HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
	HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);

	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		CC_GlobalState->readyToDriveTicks = HAL_GetTick();
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* RTD Siren & HV Startup */
	CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState | RTD_SIREN_MASK | HV_STARTUP;
	PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(CC_GlobalState->pdmTrackState);
	CAN_TxHeaderTypeDef sirenHeader =
	{
			.ExtId = rtdSiren.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = sizeof(rtdSiren.data),
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(&hcan2, &sirenHeader, rtdSiren.data, &CC_GlobalState->CAN2_TxMailbox);

	return;
}

state_t drivingState = {&state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{

	/* If AMS Contactors Closed & BMS' Healthy */

	/* Enable all channels on PDM */
	// TODO Fix Bitwise Flip on enter IDLE State under current PDM Startup Sequence

	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		CC_GlobalState->tractiveActive = true;
		CC_GlobalState->faultDetected = false;
		CC_GlobalState->rtdLightActive = true;

		memset(CC_GlobalState->rollingBrakeValues, 0, 10*sizeof(uint32_t));
		memset(CC_GlobalState->secondaryRollingBrakeValues, 0, 10*sizeof(uint32_t));
		memset(CC_GlobalState->rollingAccelValues, 0, 10*sizeof(uint32_t));
		memset(CC_GlobalState->secondaryRollingAccelValues, 0, 10*sizeof(uint32_t));
		memset(CC_GlobalState->tertiaryRollingAccelValues, 0, 10*sizeof(uint32_t));

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

		osSemaphoreRelease(CC_GlobalState->sem);
	}
	/* Start Polling ADC */
	HAL_ADC_Start_DMA(&hadc2, CC_GlobalState->brakeAdcValues, 100);
	HAL_ADC_Start_DMA(&hadc1, CC_GlobalState->accelAdcValues, 150);

	/* Run MicroBasic Script on Inverter */
	CC_SetBool_t runLeftScript = Compose_CC_SetBool(INVERTER_LEFT_NODE_ID, 0x01, 0xFFFFFFFF);
	CAN_TxHeaderTypeDef runLeftHeader =
	{
			.StdId = runLeftScript.id,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = sizeof(runLeftScript.data),
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(&CAN_1, &runLeftHeader, runLeftScript.data, &CC_GlobalState->CAN1_TxMailbox);

	CC_SetBool_t runRightScript = Compose_CC_SetBool(INVERTER_RIGHT_NODE_ID, 0x01, 0xFFFFFFFF);
	CAN_TxHeaderTypeDef runRightHeader =
	{
			.StdId = runRightScript.id,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = sizeof(runRightScript.data),
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(&CAN_1, &runRightHeader, runRightScript.data, &CC_GlobalState->CAN1_TxMailbox);
	return;
}


void state_driving_iterate(fsm_t *fsm)
{

	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Flash RTD */
		if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) > 1000)
		{
			HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, !CC_GlobalState->rtdLightActive);
			CC_GlobalState->rtdLightActive = !CC_GlobalState->rtdLightActive;
			CC_GlobalState->readyToDriveTicks = HAL_GetTick();

			CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState & (~RTD_SIREN_MASK);
			PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(CC_GlobalState->pdmTrackState);
			CAN_TxHeaderTypeDef sirenHeader =
			{
					.ExtId = rtdSiren.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = sizeof(rtdSiren.data),
					.TransmitGlobalTime = DISABLE,
			};
			HAL_CAN_AddTxMessage(&hcan2, &sirenHeader, rtdSiren.data, &CC_GlobalState->CAN2_TxMailbox);
		}
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->amsTicks) > 100 && !CC_GlobalState->AMS_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100 && !CC_GlobalState->SHDN_1_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100 && !CC_GlobalState->Inverter_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets on CAN1 */
	while(osMessageQueueGetCount(CC_GlobalState->CAN1Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN1Queue, &msg, 0U, 0U) == osOK)
		{
			if(msg.header.IDE == CAN_ID_STD) {
				/* Inverter Heartbeat */
				if(msg.header.StdId == 0x700+INVERTER_LEFT_NODE_ID)
				{
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
				/* Inverter Response Packet */
				else if(msg.header.StdId == 0x580+INVERTER_LEFT_NODE_ID)
				{
					char x[80];
					int len;
					/* Motor RPM Response Packet */
					if((msg.data[2] << 8 | msg.data[1]) == 0x210A)
					{
						/* Parse Motor RPM */
						int16_t motorRPM = 0;
						Parse_CC_RequestRPM(msg.data, &motorRPM);

						/* Echo Motor RPM */
						len = sprintf(x, "[%li] Got RPM from CAN1: %i\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, motorRPM);
						//CC_LogInfo(x, len);
					}
					else
					{
						/* Echo CAN Packet if index not recognised */
						len = sprintf(x, "[%li] Got CAN msg from CAN1: %02lX\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, msg.header.StdId);
						//CC_LogInfo(x, len);
					}
				}
			}
		}
	}

	/* Check for Queued CAN Packets on CAN2 */
	while(osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK)
		{
			/* Packet Handler */
			/* AMS Heartbeat */
			if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					bool initialised; bool HVAn; bool HVBn; bool precharge; bool HVAp; bool HVBp; uint16_t averageVoltage; uint16_t runtime;
					Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn, &precharge, &HVAp, &HVBp, &averageVoltage, &runtime);
					CC_GlobalState->amsTicks = HAL_GetTick();
					CC_GlobalState->amsInit = initialised;
					osSemaphoreRelease(CC_GlobalState->sem);
				}
			}
			/* Shutdown 1 Heartbeat */
			else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					uint8_t segmentState;
					Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
					CC_GlobalState->shutdownOneTicks = HAL_GetTick();
					osSemaphoreRelease(CC_GlobalState->sem);
				}
			}
			/* Shutdown IMD Heartbeat */
			else if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					uint8_t pwmState;
					Parse_SHDN_IMD_HeartbeatResponse(*((SHDN_IMD_HeartbeatResponse_t*)&(msg.data)), &pwmState);
					CC_GlobalState->shutdownImdTicks = HAL_GetTick();
					osSemaphoreRelease(CC_GlobalState->sem);
				}
			}
			/* Shutdown Triggered Fault */
			else if(msg.header.ExtId == Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0))
			{
				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
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
	char x[80]; uint32_t len;

	/* Echo ADC Failure for Debugging */
	if(CC_GlobalState->faultDetected)
	{
		//CC_LogInfo("ADC Fault Detected\r\n", strlen("ADC Fault Detected\r\n"));
	}
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Brake Travel Record & Sum 10 Values */
		for (int i=0; i < 10; i++)
		{
			if (i == 9)
			{
				CC_GlobalState->rollingBrakeValues[i] = CC_GlobalState->brakeAdcValues[0];
				CC_GlobalState->secondaryRollingBrakeValues[i] = CC_GlobalState->brakeAdcValues[1];
				CC_GlobalState->rollingAccelValues[i] = CC_GlobalState->accelAdcValues[0];
				CC_GlobalState->secondaryRollingAccelValues[i] = CC_GlobalState->accelAdcValues[1];
				CC_GlobalState->tertiaryRollingAccelValues[i] = CC_GlobalState->accelAdcValues[2];
			}
			else
			{
				CC_GlobalState->rollingBrakeValues[i] = CC_GlobalState->rollingBrakeValues[i+1];
				CC_GlobalState->secondaryRollingBrakeValues[i] = CC_GlobalState->secondaryRollingBrakeValues[i+1];
				CC_GlobalState->rollingAccelValues[i] = CC_GlobalState->rollingAccelValues[i+1];
				CC_GlobalState->secondaryRollingAccelValues[i] = CC_GlobalState->secondaryRollingAccelValues[i+1];
				CC_GlobalState->tertiaryRollingAccelValues[i] = CC_GlobalState->tertiaryRollingAccelValues[i+1];
			}
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	uint32_t brake_one_sum = 0; uint32_t brake_one_avg = 0;uint32_t brake_two_sum = 0;uint32_t brake_two_avg = 0;
	uint32_t accel_one_sum = 0; uint32_t accel_one_avg = 0; uint32_t accel_two_avg = 0; uint32_t accel_three_sum = 0; uint32_t accel_three_avg = 0;
	uint32_t accel_two_sum = 0;

	for (int i=0; i < 10; i++)
	{
		brake_one_sum += CC_GlobalState->rollingBrakeValues[i];
		brake_two_sum += CC_GlobalState->secondaryRollingBrakeValues[i];
		accel_one_sum += CC_GlobalState->rollingAccelValues[i];
		accel_two_sum += CC_GlobalState->secondaryRollingAccelValues[i];
		accel_three_sum += CC_GlobalState->tertiaryRollingAccelValues[i];
	}

	/* Average 10 Latest Brake Travel Values */
	brake_one_avg = brake_one_sum / 10;
	brake_two_avg = brake_two_sum / 10;

	accel_one_avg = accel_one_sum / 10;
	accel_two_avg = accel_two_sum / 10;
	accel_three_avg = accel_three_sum / 10;

	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Map Travel to Pedal Pos */
		brake_travel[0] = map(brake_one_avg, CC_GlobalState->brakeMin[0], CC_GlobalState->brakeMax[0], 0, MAX_DUTY_CYCLE);
		brake_travel[1] = map(brake_two_avg, CC_GlobalState->brakeMin[1], CC_GlobalState->brakeMax[1], 0, MAX_DUTY_CYCLE);

		accel_travel[0] = map(accel_one_avg, CC_GlobalState->accelMin[0], CC_GlobalState->accelMax[0], 0, MAX_DUTY_CYCLE);
		accel_travel[1] = map(accel_two_avg, CC_GlobalState->accelMin[1], CC_GlobalState->accelMax[1], 0, MAX_DUTY_CYCLE);
		accel_travel[2] = map(accel_three_avg, CC_GlobalState->accelMin[2], CC_GlobalState->accelMax[2], 0, MAX_DUTY_CYCLE);

		/* Recover for noise */
		if(!CC_GlobalState->ADC_Debug && CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0])
		{
			uint8_t brakeSensors = *(&brake_travel + 1) - brake_travel;
			uint8_t accelSensors = *(&accel_travel + 1) - accel_travel;
			bool currentFault = false;
			for (int i = 0; i < brakeSensors; i++)
			{
				for (int y = 0; y < brakeSensors; y++)
				{
					if(brake_travel[i] >= (int64_t)brake_travel[y]+POT_DESYNC || brake_travel[i] <= (int64_t)brake_travel[i]-POT_DESYNC)
					{
						currentFault = true;
					}
				}
				if (CC_GlobalState->brakeAdcValues[i] <= CC_GlobalState->brakeMin[i] - POT_DESYNC
						|| CC_GlobalState->brakeAdcValues[i] >= CC_GlobalState->brakeMax[i] + POT_DESYNC)
				{
					currentFault = true;
				}
			}
			for (int i = 0; i < accelSensors; i++)
			{
				for (int y = 0; y < accelSensors; y++)
				{
					if(accel_travel[i] >= (int64_t)accel_travel[y]+POT_DESYNC || accel_travel[i] <= (int64_t)accel_travel[i]-POT_DESYNC)
					{
						currentFault = true;
						//CC_LogInfo("ADC Desync\r\n", strlen("ADC Desync\r\n"));
					}
				}
				if (CC_GlobalState->accelAdcValues[i] <= CC_GlobalState->accelMin[i] - POT_DESYNC
						|| CC_GlobalState->accelAdcValues[i] >= CC_GlobalState->accelMax[i] + POT_DESYNC)
				{
					currentFault = true;
					//CC_LogInfo("ADC Range Error\r\n", strlen("ADC Range Error\r\n"));
				}
			}
			if(currentFault)
			{
				if(!CC_GlobalState->faultDetected)
				{
					CC_GlobalState->implausibleTicks = HAL_GetTick();
				}
				CC_GlobalState->faultDetected = true;
			}
			else if(!currentFault && CC_GlobalState->faultDetected)
			{
				CC_GlobalState->faultDetected = false;
				CC_GlobalState->implausibleTicks = 0;
			}
		}

		/* Average 2 Brake Travel Positions */
		if(CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0])
		{
			CC_GlobalState->brakeTravel = MAX_DUTY_CYCLE-((brake_travel[0]+brake_travel[1])/2);
			CC_GlobalState->accelTravel = MAX_DUTY_CYCLE-((accel_travel[0]+accel_travel[1]+accel_travel[2])/3);

			if((int32_t)CC_GlobalState->accelTravel < 0)
			{
				CC_GlobalState->accelTravel = 0;
			}
			if((int32_t)CC_GlobalState->brakeTravel < 0)
			{
				CC_GlobalState->brakeTravel = 0;
			}
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Echo Pedal Positions */
	if(CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0])
	{
		//len = sprintf(x, "Pedal Travel: %li %li\r\n", CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
		//CC_LogInfo(x, len);
	}

	/*
	 * If Throttle and Brake Implausibility State Clock < 100ms
	 * Suspend Tractive System Operations
	 */
	if(CC_GlobalState->faultDetected
			&& !CC_GlobalState->ADC_Debug
			&& CC_GlobalState->tractiveActive
			&& (HAL_GetTick() - CC_GlobalState->implausibleTicks) >= 100)
	{
		CC_GlobalState->tractiveActive = false;
		CC_LogInfo("Disabling Tractive Operations\r\n", strlen("Disabling Tractive Operations\r\n"));
	}

	/*
	 * Motor Control Commands
	 */

	/* Enable Script to Execute on Inverters */
	if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) % 60 == 0
			&& CC_GlobalState->rollingAccelValues[0] > 0
			&& CC_GlobalState->rollingBrakeValues[0])
	{
		/* Run MicroBasic Script on Inverter */
		CC_SetBool_t runLeftScript = Compose_CC_SetBool(INVERTER_LEFT_NODE_ID, 0x01, 0xFFFFFFFF);
		CAN_TxHeaderTypeDef runLeftHeader =
		{
				.StdId = runLeftScript.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &runLeftHeader, runLeftScript.data, &CC_GlobalState->CAN1_TxMailbox);

		CC_SetBool_t runRightScript = Compose_CC_SetBool(INVERTER_RIGHT_NODE_ID, 0x01, 0xFFFFFFFF);
		CAN_TxHeaderTypeDef runRightHeader =
		{
				.StdId = runRightScript.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &runRightHeader, runRightScript.data, &CC_GlobalState->CAN1_TxMailbox);
	}

	/* Send Accel Command */
	if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) % 36 == 0
			&& CC_GlobalState->faultDetected
			&& CC_GlobalState->rollingAccelValues[0] > 0
			&& CC_GlobalState->rollingBrakeValues[0])
	{
		/* Generate Desired Motor Command Value */
		len = sprintf(x, "Motor Command: %li %li\r\n", CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
		CC_LogInfo(x, len);

		/* Send Left Accel Command */
		CC_SetVariable_t accelLeftCommand = Compose_CC_SetVariable(INVERTER_LEFT_NODE_ID,
				0x01,
				CC_GlobalState->accelTravel);
		CAN_TxHeaderTypeDef accelLeftHeader =
		{
				.StdId = accelLeftCommand.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &accelLeftHeader, accelLeftCommand.data, &CC_GlobalState->CAN1_TxMailbox);

		/* Send Right Accel Command */
		CC_SetVariable_t accelRightCommand = Compose_CC_SetVariable(INVERTER_RIGHT_NODE_ID,
				0x01,
				CC_GlobalState->accelTravel);
		CAN_TxHeaderTypeDef accelRightHeader =
		{
				.StdId = accelRightCommand.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &accelRightHeader, accelRightCommand.data, &CC_GlobalState->CAN1_TxMailbox);
	}

	/* Send Brake Travel Command */
	if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) % 47 == 0
			&& CC_GlobalState->rollingAccelValues[0] > 0
			&& CC_GlobalState->rollingBrakeValues[0])
	{
		/* Send Left Brake Command */
		CC_SetVariable_t brakeLeftCommand = Compose_CC_SetVariable(INVERTER_LEFT_NODE_ID,
				0x02,
				CC_GlobalState->brakeTravel);
		CAN_TxHeaderTypeDef brakeLeftHeader =
		{
				.StdId = brakeLeftCommand.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &brakeLeftHeader, brakeLeftCommand.data, &CC_GlobalState->CAN1_TxMailbox);

		/* Send Right Brake Command */
		CC_SetVariable_t brakeRightCommand = Compose_CC_SetVariable(INVERTER_RIGHT_NODE_ID,
				0x02,
				CC_GlobalState->brakeTravel);
		CAN_TxHeaderTypeDef brakeRightHeader =
		{
				.StdId = brakeRightCommand.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &brakeRightHeader, brakeRightCommand.data, &CC_GlobalState->CAN1_TxMailbox);
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
//		HAL_CAN_AddTxMessage(&CAN_1, &accelZeroLeftHeader, accelZeroLeftCommand.data, &CC_GlobalState->CAN1_TxMailbox);
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
//		HAL_CAN_AddTxMessage(&CAN_1, &accelZeroRightHeader, accelZeroRightCommand.data, &CC_GlobalState->CAN1_TxMailbox);
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
//		HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
//		HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
//		HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
//		fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");
//	}

	/*
	 * If 500ms has exceeded since SoC Request
	 * Request State of Charge
	 */

	if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) % 27 == 0
			&& CC_GlobalState->rollingAccelValues[0] > 0
			&& CC_GlobalState->rollingBrakeValues[0])
	{
		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			if(CC_GlobalState->brakeTravel > 10)
			{
				CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState | BRAKE_LIGHT_MASK;

			}
			else
			{
				CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState & (~BRAKE_LIGHT_MASK);
			}
			osSemaphoreRelease(CC_GlobalState->sem);
		}
		PDM_SetChannelStates_t brakeLightState = Compose_PDM_SetChannelStates(CC_GlobalState->pdmTrackState);
		CAN_TxHeaderTypeDef brakeHeader =
		{
				.ExtId = brakeLightState.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(brakeLightState.data),
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&hcan2, &brakeHeader, brakeLightState.data, &CC_GlobalState->CAN2_TxMailbox);
	}
}

void state_driving_exit(fsm_t *fsm)
{
	/* Broadcast Soft Shutdown */
	return;
}

state_t debugState = {&state_debug_enter, &state_debug_iterate, &state_debug_exit, "Debug_s"};

void state_debug_enter(fsm_t *fsm)
{
	PDM_InitiateStartup_t init = Compose_PDM_InitiateStartup();
	uint8_t test[1] = {0};
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = init.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = sizeof(test),
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(&hcan2, &header, test, &CC_GlobalState->CAN2_TxMailbox);
	osDelay(100);
	return;
}

void state_debug_iterate(fsm_t *fsm)
{
	for(int i = 1; i <= 32; i++)
	{
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(1 << i);
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = pdmStartup.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(pdmStartup.data),
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&hcan2, &header, pdmStartup.data, &CC_GlobalState->CAN2_TxMailbox);
		osDelay(100);
	}

	return;
}

void state_debug_exit(fsm_t *fsm)
{
	//CC_LogInfo("Exit Debugging\r\n", strlen("Exit Debugging\r\n"));
	return;
}

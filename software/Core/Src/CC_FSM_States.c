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

#define BRAKE_PEDAL_ONE_MIN 2360
#define BRAKE_PEDAL_ONE_MAX 3170
#define BRAKE_PEDAL_TWO_MIN 2280
#define BRAKE_PEDAL_TWO_MAX 3110

#define ACCEL_PEDAL_ONE_MIN 890
#define ACCEL_PEDAL_ONE_MAX 3350
#define ACCEL_PEDAL_TWO_MIN 910
#define ACCEL_PEDAL_TWO_MAX 3400
#define ACCEL_PEDAL_THREE_MIN 910
#define ACCEL_PEDAL_THREE_MAX 3380

#define CAN_1 hcan1
#define CAN_2 hcan2
#define CAN_3 hcan3

#define INVERTER_1_NODE_ID 100

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

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
			CC_GlobalState->PDM_Debug = true;
			CC_GlobalState->AMS_Debug = false;
			CC_GlobalState->ADC_Debug = false;
			CC_GlobalState->SHDN_Debug = false;
			CC_GlobalState->SHDN_IMD_Debug = true;
			CC_GlobalState->RTD_Debug = true;
			CC_GlobalState->Inverter_Debug = true;
			CC_GlobalState->tractiveActive = false;
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
	PDM_InitiateStartup_t pdmStartup = Compose_PDM_InitiateStartup();
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = pdmStartup.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};
	uint8_t data[1] = {0xF};
	HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);

	/* Debug Tracing */
	//CC_LogInfo("Enter Start\r\n", strlen("Enter Start\r\n"));
	return;
}

void state_start_iterate(fsm_t *fsm)
{
	/* Skip boot if PDM Debugging Enabled */
	bool boot = CC_GlobalState->PDM_Debug;
	uint32_t getPowerChannels = 0; uint32_t setPowerChannels = 0;

	/* Monitor CAN Queue */
	while(osMessageQueueGetCount(CC_GlobalState->CAN2Queue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK)
		{
			/* If Startup Ok */
			if(msg.header.ExtId == Compose_CANId(0x2, 0x14, 0x0, 0x3, 0x00, 0x0))
			{
				/* Get Power Channel Values at Boot */
				getPowerChannels = 0;
				Parse_PDM_StartupOk(msg.data, &getPowerChannels);

				/* Initialise Boot with Bitwise OR on Power Channels */
				boot = true;
			}
		}
	}

	if(boot)
	{
		/* Set Power Channel Values to Enable on Start */
		setPowerChannels |= 1 << getPowerChannels;
		PDM_SelectStartup_t pdmStartup = Compose_PDM_SelectStartup(setPowerChannels);
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
			CC_GlobalState->shutdownTicks = HAL_GetTick();
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
	int brake_threshold_range = BRAKE_PRESSURE_MAX - BRAKE_PRESSURE_MIN;
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
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Shutdown Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownTicks) > 100 && !CC_GlobalState->SHDN_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100 && !CC_GlobalState->Inverter_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
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
				if(msg.header.StdId == 0x764)
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
				/* Shutdown Heartbeat */
				else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0))
				{
					if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						uint8_t segmentState;
						Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
						CC_GlobalState->shutdownTicks = HAL_GetTick();
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
							&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
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
	if(raw > CC_GlobalState->brakePressureThreshold && CC_GlobalState->amsInit && CC_GlobalState->ccInit)
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
	/* Else */

	/* Hard Shutdown Power Off */
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
		}

		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->amsTicks) > 100 && !CC_GlobalState->AMS_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Shutdown Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownTicks) > 100 && !CC_GlobalState->SHDN_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100 && !CC_GlobalState->Inverter_Debug)
		{
			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
					&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
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
				if(msg.header.StdId == 0x764)
				{
					CC_GlobalState->inverterTicks = HAL_GetTick();
				}
				/* Inverter Response Packet */
				else if(msg.header.StdId == 0x580+INVERTER_1_NODE_ID)
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
						//len = sprintf(x, "[%li] Got CAN RPM from CAN1: %i\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, motorRPM);
						//CC_LogInfo(x, len);

						/* Generate Desired Motor Command Value */
						int32_t motorCommandValue = map(CC_GlobalState->accelTravel, 0, 100, 0, 200);
						//len = sprintf(x, "Motor Command: %i %li\r\n", CC_GlobalState->accelTravel, motorCommand);
						//CC_LogInfo(x, len);

						/* Send Motor Command to First Motor */
						CC_MotorCommand_t MotorCommandOne = Compose_CC_MotorCommand(INVERTER_1_NODE_ID,
								motorCommandValue,
								MOTOR_1_SUBINDEX);
						CAN_TxHeaderTypeDef firstHeader =
						{
								.StdId = MotorCommandOne.id,
								.IDE = CAN_ID_STD,
								.RTR = CAN_RTR_DATA,
								.DLC = 8,
								.TransmitGlobalTime = DISABLE,
						};
						HAL_CAN_AddTxMessage(&CAN_1, &firstHeader, MotorCommandOne.data, &CC_GlobalState->CAN1_TxMailbox);

						/* Send Motor Command to Second Motor */
						CC_MotorCommand_t MotorCommandTwo = Compose_CC_MotorCommand(INVERTER_1_NODE_ID,
								motorCommandValue,
								MOTOR_2_SUBINDEX);
						CAN_TxHeaderTypeDef secondHeader =
						{
								.StdId = MotorCommandTwo.id,
								.IDE = CAN_ID_STD,
								.RTR = CAN_RTR_DATA,
								.DLC = 8,
								.TransmitGlobalTime = DISABLE,
						};
						HAL_CAN_AddTxMessage(&CAN_1, &secondHeader, MotorCommandTwo.data, &CC_GlobalState->CAN1_TxMailbox);
					}
					else{
						/* Echo CAN Packet if index not recognised */
						len = sprintf(x, "[%li] Got CAN msg from CAN1: %02lX\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, msg.header.StdId);
						CC_LogInfo(x, len);
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
			/* Shutdown Heartbeat */
			else if(msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					uint8_t segmentState;
					Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*)&(msg.data)), &segmentState);
					CC_GlobalState->shutdownTicks = HAL_GetTick();
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
				// TODO DEAL WITH INVERTERS HERE WITH SOFT INVERTER SHUTDOWN
				CC_ShutdownInverter_t shutdownInverter = Compose_CC_ShutdownInverter(INVERTER_1_NODE_ID);
				CAN_TxHeaderTypeDef header =
				{
						.StdId = shutdownInverter.id,
						.IDE = CAN_ID_STD,
						.RTR = CAN_RTR_DATA,
						.DLC = 8,
						.TransmitGlobalTime = DISABLE,
				};
				HAL_CAN_AddTxMessage(&CAN_1, &header, shutdownInverter.data, &CC_GlobalState->CAN1_TxMailbox);

				CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault\r\n", true,
						&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox,
						&CC_GlobalState->CAN3_TxMailbox, &CAN_1, &CAN_2, &CAN_3, &huart3);
				fsm_changeState(fsm, &idleState, "Resetting to Idle to Clean");
			}
		}
	}

	/*
	 * Read 3 Throttle ADC Values
	 * Read 2 Brake ADC Values
	 */
	uint16_t brake_travel_one; uint16_t brake_travel_two;
	uint16_t accel_travel_one; uint16_t accel_travel_two; uint16_t accel_travel_three;
	char x[80]; uint32_t len;

	/* Echo ADC Failure for Debugging */
	if(CC_GlobalState->faultDetected)
	{
		CC_LogInfo("ADC Fault Detected\r\n", strlen("ADC Fault Detected\r\n"));
	}
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Check for non-expected ADC Values
		 * Trigger Fault outside expected range
		 * Power trip, surge to sensor etc.
		 */
		if(!CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug)
		{
			for (int i = 0; i < 2; i++) {
				if (CC_GlobalState->brakeAdcValues[i] <= CC_GlobalState->brakeMin[i] - 100
						|| CC_GlobalState->brakeAdcValues[i] >= CC_GlobalState->brakeMax[i] + 100)
				{
					CC_LogInfo("Brake ADC Tripped\r\n", strlen("Brake ADC Tripped\r\n"));
					CC_GlobalState->faultDetected = true;
					CC_GlobalState->implausibleTicks = HAL_GetTick();
				}
			}
			for (int i = 0; i < 3; i++) {
				if (CC_GlobalState->accelAdcValues[i] <= CC_GlobalState->accelMin[i] - 100
						|| CC_GlobalState->accelAdcValues[i] >= CC_GlobalState->accelMax[i] + 100)
				{
					CC_LogInfo("Accel ADC Tripped\r\n", strlen("Accel ADC Tripped\r\n"));
					CC_GlobalState->faultDetected = true;
					CC_GlobalState->implausibleTicks = HAL_GetTick();
				}
			}
		}

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
		if(!CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug)
		{
			/* Check for New Min/Max Brake Values */
			if(CC_GlobalState->rollingBrakeValues[0] > 0 && CC_GlobalState->secondaryRollingBrakeValues[0] > 0)
			{
				if(brake_one_avg <= CC_GlobalState->brakeMin[0])
				{
					CC_GlobalState->brakeMin[0] = brake_one_avg;
				}
				if(brake_one_avg >= CC_GlobalState->brakeMax[0])
				{
					CC_GlobalState->brakeMax[0] = brake_one_avg;
				}
				if(brake_two_avg <= CC_GlobalState->brakeMin[1])
				{
					CC_GlobalState->brakeMin[1] = brake_two_avg;
				}
				if(brake_two_avg >= CC_GlobalState->brakeMax[1])
				{
					CC_GlobalState->brakeMax[1] = brake_two_avg;
				}
			}
			if(CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->secondaryRollingAccelValues[0] > 0 && CC_GlobalState->tertiaryRollingAccelValues[0] > 0)
			{
				if(accel_one_avg <= CC_GlobalState->accelMin[0])
				{
					CC_GlobalState->accelMin[0] = accel_one_avg;
				}
				if(accel_one_avg >= CC_GlobalState->accelMax[0])
				{
					CC_GlobalState->accelMax[0] = accel_one_avg;
				}
				if(accel_two_avg <= CC_GlobalState->accelMin[1])
				{
					CC_GlobalState->accelMin[1] = accel_two_avg;
				}
				if(accel_two_avg >= CC_GlobalState->accelMax[1])
				{
					CC_GlobalState->accelMax[1] = accel_two_avg;
				}
				if(accel_three_avg <= CC_GlobalState->accelMin[2])
				{
					CC_GlobalState->accelMin[2] = accel_three_avg;
				}
				if(accel_three_avg >= CC_GlobalState->accelMax[2])
				{
					CC_GlobalState->accelMax[2] = accel_three_avg;
				}
			}
		}

		/* Map Travel to Pedal Pos */
		brake_travel_one = map(brake_one_avg, CC_GlobalState->brakeMin[0]+2, CC_GlobalState->brakeMax[0]-5, 0, 100);
		brake_travel_two = map(brake_two_avg, CC_GlobalState->brakeMin[1]+2, CC_GlobalState->brakeMax[1]-5, 0, 100);

		accel_travel_one = map(accel_one_avg, CC_GlobalState->accelMin[0]+2, CC_GlobalState->accelMax[0]-6, 0, 100);
		accel_travel_two = map(accel_two_avg, CC_GlobalState->accelMin[1]+2, CC_GlobalState->accelMax[1]-6, 0, 100);
		accel_travel_three = map(accel_three_avg, CC_GlobalState->accelMin[2]+2, CC_GlobalState->accelMax[2]-6, 0, 100);

		/* Ensure Brake & Accel Pots Synced */
		if(!CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug
				&& CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0]
																								   && (brake_travel_one >= brake_travel_two+10
																										   || brake_travel_one <= brake_travel_two-10))
		{
			CC_LogInfo("Brake ADC Desync\r\n", strlen("Brake ADC Desync\r\n"));
			CC_GlobalState->faultDetected = true;
			CC_GlobalState->implausibleTicks = HAL_GetTick();
		}
		if(!CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug
				&& CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0]
																								   && (accel_travel_one >= accel_travel_two+10
																										   || accel_travel_one <= accel_travel_two-10
																										   || accel_travel_one >= accel_travel_three+10
																										   || accel_travel_one <= accel_travel_three-10
																										   || accel_travel_two >= accel_travel_three+10
																										   || accel_travel_two <= accel_travel_three-10))
		{
			CC_LogInfo("Accel ADC Desync\r\n", strlen("Accel ADC Desync\r\n"));
			CC_GlobalState->faultDetected = true;
			CC_GlobalState->implausibleTicks = HAL_GetTick();
		}

		/* Average 2 Brake Travel Positions */
		if(CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0])
		{
			CC_GlobalState->brakeTravel = 100-((brake_travel_one+brake_travel_two)/2);
			CC_GlobalState->accelTravel = 100-((accel_travel_one+accel_travel_two+accel_travel_three)/3);
		}

		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Echo Pedal Positions */
	if(CC_GlobalState->rollingAccelValues[0] > 0 && CC_GlobalState->rollingBrakeValues[0])
	{
		len = sprintf(x, "Pedal Positions: %i %i\r\n", CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
		CC_LogInfo(x, len);
	}

	/*
	 * Read Steering Angle ADC Value
	 * Log Steering Angle
	 */

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
	 * Call Torque Vectoring Algorithm
	 */

	/*
	 * Calculate Regen
	 */

	/*
	 * Request RPM from Motors
	 */
	if(!CC_GlobalState->Inverter_Debug
			&& CC_GlobalState->tractiveActive
			&& (HAL_GetTick() - CC_GlobalState->readyToDriveTicks) % 100 == 0)
	{
		/* Broadcast Motor RPM Request on CAN1 */
		CC_RequestRPM_t requestRPM = Compose_CC_RequestRPM(INVERTER_1_NODE_ID);
		CAN_TxHeaderTypeDef header =
		{
				.StdId = requestRPM.id,
				.IDE = CAN_ID_STD,
				.RTR = CAN_RTR_DATA,
				.DLC = 8,
				.TransmitGlobalTime = DISABLE,
		};
		HAL_CAN_AddTxMessage(&CAN_1, &header, requestRPM.data, &CC_GlobalState->CAN1_TxMailbox);
	}

	/*
	 * If Throttle or Brake Implausibility State Clock > 1000ms
	 * Engage Soft Shutdown (Reset to Idle)
	 */
	if(CC_GlobalState->faultDetected && !CC_GlobalState->ADC_Debug && !CC_GlobalState->tractiveActive && (HAL_GetTick() - CC_GlobalState->implausibleTicks) >= 1000)
	{
		/* Broadcast Soft Shutdown on all CAN lines */
		CC_SoftShutdown_t softShutdown = Compose_CC_SoftShutdown();
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = softShutdown.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = 1,
				.TransmitGlobalTime = DISABLE,
		};
		uint8_t data[1] = {0xF};
		HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
		HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
		fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");
	}

	/*
	 * If 500ms has exceeded since SoC Request
	 * Request State of Charge
	 */
}

void state_driving_exit(fsm_t *fsm)
{
	/* Broadcast Soft Shutdown */
	return;
}

state_t debugState = {&state_debug_enter, &state_debug_iterate, &state_debug_exit, "Debug_s"};

void state_debug_enter(fsm_t *fsm)
{
	CC_LogInfo("Enter Debugging\r\n", strlen("Enter Debugging\r\n"));
	return;
}

void state_debug_iterate(fsm_t *fsm)
{
	/* Broadcast Motor RPM Request on CAN1 */
	CC_RequestRPM_t requestRPM = Compose_CC_RequestRPM(INVERTER_1_NODE_ID);
	CAN_TxHeaderTypeDef header =
	{
			.StdId = requestRPM.id,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = 8,
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(&CAN_1, &header, requestRPM.data, &CC_GlobalState->CAN1_TxMailbox);
	return;
}

void state_debug_exit(fsm_t *fsm)
{
	CC_LogInfo("Exit Debugging\r\n", strlen("Exit Debugging\r\n"));
	return;
}

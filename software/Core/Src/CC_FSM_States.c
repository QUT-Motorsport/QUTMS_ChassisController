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

#define BRAKE_PEDAL_ONE_MIN 320
#define BRAKE_PEDAL_ONE_MAX 3400
#define BRAKE_PEDAL_TWO_MIN 240
#define BRAKE_PEDAL_TWO_MAX 3320

/* Util Functions */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

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
		CC_GlobalState = malloc(sizeof(CC_GlobalState_t));
		memset(CC_GlobalState, 0, sizeof(CC_GlobalState_t));

		// As CC_GlobalState is accessible across threads, we need to use a semaphore to access it
		CC_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);
		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			/* Bind and configure initial global states */
			CC_GlobalState->PDM_Debug = true;
			CC_GlobalState->AMS_Debug = false;
			CC_GlobalState->SHDN_IMD_Debug = true;
			CC_GlobalState->RTD_Debug = true;

			CC_GlobalState->tractiveActive = true;

			//CC_GlobalState->brakePressure;
			memset(CC_GlobalState->rollingBrakeValues, 0, 10*sizeof(uint16_t));
			memset(CC_GlobalState->secondaryRollingBrakeValues, 0, 10*sizeof(uint16_t));
			CC_GlobalState->brakeOneMin = BRAKE_PEDAL_ONE_MIN;
			CC_GlobalState->brakeOneMax = BRAKE_PEDAL_ONE_MAX;
			CC_GlobalState->brakeTwoMin = BRAKE_PEDAL_TWO_MIN;
			CC_GlobalState->brakeTwoMax = BRAKE_PEDAL_TWO_MAX;

			osSemaphoreRelease(CC_GlobalState->sem);
		}

		CC_GlobalState->CANQueue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
		if(CC_GlobalState->CANQueue == NULL)
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
	CC_LogInfo("Enter Start\r\n", strlen("Enter Start\r\n"));
	return;
}

void state_start_iterate(fsm_t *fsm)
{
	/* Skip boot if PDM Debugging Enabled */
	bool boot = CC_GlobalState->PDM_Debug;
	uint32_t getPowerChannels = 0; uint32_t setPowerChannels = 0;

	/* Monitor CAN Queue */
	while(osMessageQueueGetCount(CC_GlobalState->CANQueue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/* If Startup Ok */
			if(msg.header.ExtId == Compose_CANId(0x2, 0x14, 0x0, 0x3, 0x00, 0x0))
			{
				/* Get Power Channel Values at Boot */
				getPowerChannels = 0;
				Parse_PDM_StartupOk(*((PDM_StartupOk_t*)&(msg.data)), &getPowerChannels);

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
			CC_GlobalState->shutdownImdTicks = HAL_GetTick();
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		/* Engage Idle State (Waiting for RTD) */
		fsm_changeState(fsm, &idleState, "PDM Boot Sequence Initiated");
	}
	return;
}

void state_start_exit(fsm_t *fsm)
{
	/* Wake/Ready to Idle over CAN */
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
		CC_GlobalState->brakeThreshold = BRAKE_PRESSURE_MIN + (0.2 * brake_threshold_range);
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
			CC_LogInfo("Fatal Shutdown AMS\r\n", strlen("Fatal Shutdown AMS\r\n"));
			CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
			CAN_TxHeaderTypeDef header =
			{
					.ExtId = fatalShutdown.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = 1,
					.TransmitGlobalTime = DISABLE,
			};
			uint8_t data[1] = {0xF};
			HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
			CAN_TxHeaderTypeDef header =
			{
					.ExtId = fatalShutdown.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = 1,
					.TransmitGlobalTime = DISABLE,
			};
			uint8_t data[1] = {0xF};
			HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets */
	while(osMessageQueueGetCount(CC_GlobalState->CANQueue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/* Packet Handler */
			/* AMS Heartbeat */
			if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					bool HVAn; bool HVBn; bool precharge; bool HVAp; bool HVBp; uint16_t averageVoltage; uint16_t runtime;
					Parse_AMS_HeartbeatResponse(*((AMS_HeartbeatResponse_t*)&(msg.data)), &HVAn, &HVBn, &precharge, &HVAp, &HVBp, &averageVoltage, &runtime);
					CC_GlobalState->amsTicks = HAL_GetTick();
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
		HAL_ADC_Start(&hadc1);
		raw = HAL_ADC_GetValue(&hadc1);
	}
	if(raw > CC_GlobalState->brakeThreshold)
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

	/* Play RTD Siren for 2 Seconds */

	/* Enable all channels on PDM */
	// TODO Fix Bitwise Flip on enter IDLE State under current PDM Startup Sequence

	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		CC_GlobalState->tractiveActive = true;
		CC_GlobalState->rtdLightActive = true;
		osSemaphoreRelease(CC_GlobalState->sem);
	}
	/* Start Polling ADC */
	HAL_ADC_Start_DMA(&hadc2, adcValues, 500);
	/* Else */

	/* Hard Shutdown Power Off */
	return;
}


void state_driving_iterate(fsm_t *fsm)
{
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Flash RTD */
		if((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) > 100)
		{
			if(!CC_GlobalState->rtdLightActive)
			{
				HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);
				CC_GlobalState->rtdLightActive = true;
			}
			else
			{
				HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
				CC_GlobalState->rtdLightActive = false;
			}
			CC_GlobalState->readyToDriveTicks = HAL_GetTick();
		}

		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->amsTicks) > 100 && !CC_GlobalState->AMS_Debug)
		{
			CC_LogInfo("Fatal Shutdown AMS Driving\r\n", strlen("Fatal Shutdown AMS Driving\r\n"));
			CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
			CAN_TxHeaderTypeDef header =
			{
					.ExtId = fatalShutdown.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = 1,
					.TransmitGlobalTime = DISABLE,
			};
			uint8_t data[1] = {0xF};
			HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100 && !CC_GlobalState->SHDN_IMD_Debug)
		{
			CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
			CAN_TxHeaderTypeDef header =
			{
					.ExtId = fatalShutdown.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = 1,
					.TransmitGlobalTime = DISABLE,
			};
			uint8_t data[1] = {0xF};
			HAL_CAN_AddTxMessage(&hcan1, &header, data, &CC_GlobalState->CAN1_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan3, &header, data, &CC_GlobalState->CAN3_TxMailbox);
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/* Check for Queued CAN Packets */
	while(osMessageQueueGetCount(CC_GlobalState->CANQueue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/* Packet Handler */
			/* AMS Heartbeat */
			if(msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0))
			{
				if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					bool HVAn; bool HVBn; bool precharge; bool HVAp; bool HVBp; uint16_t averageVoltage; uint16_t runtime;
					Parse_AMS_HeartbeatResponse(*((AMS_HeartbeatResponse_t*)&(msg.data)), &HVAn, &HVBn, &precharge, &HVAp, &HVBp, &averageVoltage, &runtime);
					CC_GlobalState->amsTicks = HAL_GetTick();
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
		}
	}

	/*
	 * Read 3 Throttle ADC Values
	 * Read 2 Brake ADC Values
	 */
	uint32_t brake_one_sum; uint32_t brake_one_avg; uint32_t brake_two_sum; uint32_t brake_two_avg;
	uint16_t brake_travel_one; uint16_t brake_travel_two;
	char x[80];
	uint32_t len;
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Echo ADC Failure for Debugging */
		if(!CC_GlobalState->tractiveActive)
		{
			CC_LogInfo("ADC Brake Failure\r\n", strlen("ADC Brake Failure\r\n"));
		}

		/* Check for non-expected ADC Values (Revoke Tractive System Active Status) */
		if(adcValues[0] <= CC_GlobalState->brakeOneMin - 100 || adcValues[0] >= CC_GlobalState->brakeOneMax + 100 || adcValues[1] <= CC_GlobalState->brakeTwoMin - 100 || adcValues[1] >= CC_GlobalState->brakeTwoMax + 100)
		{
			CC_GlobalState->tractiveActive = false;

			/* Debug the Fault*/
			CC_LogInfo("Failure\r\n", strlen("Failure\r\n"));
			len = sprintf(x, "Data: %li %li\r\n", adcValues[0], adcValues[1]);
			CC_LogInfo(x, len);
		}

		/* Brake Travel Record & Sum 10 Values */
		for (int i=0; i < 10; i++)
		{
			if (i == 9)
			{
				CC_GlobalState->rollingBrakeValues[i] = adcValues[0];
				CC_GlobalState->secondaryRollingBrakeValues[i] = adcValues[1];
			}
			else
			{
				CC_GlobalState->rollingBrakeValues[i] = CC_GlobalState->rollingBrakeValues[i+1];
				CC_GlobalState->secondaryRollingBrakeValues[i] = CC_GlobalState->secondaryRollingBrakeValues[i+1];
			}
			brake_one_sum += CC_GlobalState->rollingBrakeValues[i];
			brake_two_sum += CC_GlobalState->secondaryRollingBrakeValues[i];
		}

		/* Average 10 Latest Brake Travel Values */
		brake_one_avg = brake_one_sum / 10;
		brake_two_avg = brake_two_sum / 10;

		/* Check for New Min/Max Brake Values */
		if(CC_GlobalState->rollingBrakeValues[0] > 0 && CC_GlobalState->secondaryRollingBrakeValues[0] > 0)
		{
			if(brake_one_avg <= CC_GlobalState->brakeOneMin && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeOneMin = brake_one_avg;
			}
			if(brake_one_avg >= CC_GlobalState->brakeOneMax && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeOneMax = brake_one_avg;
			}
			if(brake_two_avg <= CC_GlobalState->brakeTwoMin && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeTwoMin = brake_two_avg;
			}
			if(brake_two_avg >= CC_GlobalState->brakeTwoMax && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeTwoMax = brake_two_avg;
			}
		}

		/* Map Travel to Pedal Pos */
		brake_travel_one = map(brake_one_avg, CC_GlobalState->brakeOneMin+2, CC_GlobalState->brakeOneMax-5, 0, 100);
		brake_travel_two = map(brake_two_avg, CC_GlobalState->brakeTwoMin+2, CC_GlobalState->brakeTwoMax-5, 0, 100);

		/* Ensure Brake Pots Synced */
		if(brake_travel_one >= brake_travel_two+10 || brake_travel_one <= brake_travel_two-10)
		{
			CC_GlobalState->tractiveActive = false;
		}

		/* Average 2 Brake Travel Positions */
		uint16_t brake_travel = (brake_travel_one+brake_travel_two)/2;

		/* Echo Brake Position */
		//		if(CC_GlobalState->rollingBrakeValues[0] > 0 && CC_GlobalState->secondaryRollingBrakeValues[0] > 0 && CC_GlobalState->tractiveActive)
		//		{
		//			len = sprintf(x, "Data: %li\r\n", brake_travel);
		//			CC_LogInfo(x, len);
		//		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}

	/*
	 * Calculate Throttle Implausibility
	 * Still implausible with only 2 of the 3 ADC values?
	 * Calculate Brake Implausibility
	 */

	/*
	 * Average Throttle Values to Position
	 * Average Brake Values to Position
	 */

	/*
	 * Log Throttle Position
	 * Log Brake Position
	 */

	/*
	 * Read Steering Angle ADC Value
	 * Log Steering Angle
	 */

	/* If Throttle and Brake Implausibility State Clock < 100ms */

	/*
	 * Call Torque Vectoring Algorithm
	 */

	/*
	 * Calculate Regen
	 */

	/*
	 * Send Desired Accel to Inverters
	 */

	/*
	 * If Throttle or Brake Implausibility State Clock > 1000ms
	 * Engage Soft Shutdown (Reset to Idle)
	 */
	//fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");

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
	HAL_ADC_Start_DMA(&hadc2, adcValues, 500);
	return;
}

void state_debug_iterate(fsm_t *fsm)
{
	uint32_t brake_one_sum = 0; uint32_t brake_one_avg = 0;
	uint32_t brake_two_sum = 0; uint32_t brake_two_avg = 0;
	uint16_t brake_travel_one; uint16_t brake_travel_two;
	char x[80];
	int len;
	if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		/* Echo ADC Failure for Debugging */
		if(!CC_GlobalState->tractiveActive)
		{
			CC_LogInfo("ADC Brake Failure\r\n", strlen("ADC Brake Failure\r\n"));
		}

		/* Check for non-expected ADC Values (Revoke Tractive System Active Status) */
		if(adcValues[0] <= CC_GlobalState->brakeOneMin - 100 || adcValues[0] >= CC_GlobalState->brakeOneMax + 100 || adcValues[1] <= CC_GlobalState->brakeTwoMin - 100 || adcValues[1] >= CC_GlobalState->brakeTwoMax + 100)
		{
			CC_GlobalState->tractiveActive = false;

			/* Debug the Fault*/
			//			CC_LogInfo("Failure\r\n", strlen("Failure\r\n"));
			//			len = sprintf(x, "Data: %li %li\r\n", adcValues[0], adcValues[1]);
			//			CC_LogInfo(x, len);
		}

		/* Brake Travel Record & Sum 10 Values */
		for (int i=0; i < 10; i++)
		{
			if (i == 9)
			{
				CC_GlobalState->rollingBrakeValues[i] = adcValues[0];
				CC_GlobalState->secondaryRollingBrakeValues[i] = adcValues[1];
			}
			else
			{
				CC_GlobalState->rollingBrakeValues[i] = CC_GlobalState->rollingBrakeValues[i+1];
				CC_GlobalState->secondaryRollingBrakeValues[i] = CC_GlobalState->secondaryRollingBrakeValues[i+1];
			}
			brake_one_sum += CC_GlobalState->rollingBrakeValues[i];
			brake_two_sum += CC_GlobalState->secondaryRollingBrakeValues[i];
		}

		/* Average 10 Latest Brake Travel Values */
		brake_one_avg = brake_one_sum / 10;
		brake_two_avg = brake_two_sum / 10;

		/* Check for New Min/Max Brake Values */
		if(CC_GlobalState->rollingBrakeValues[0] > 0 && CC_GlobalState->secondaryRollingBrakeValues[0] > 0)
		{
			if(brake_one_avg <= CC_GlobalState->brakeOneMin && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeOneMin = brake_one_avg;
			}
			if(brake_one_avg >= CC_GlobalState->brakeOneMax && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeOneMax = brake_one_avg;
			}
			if(brake_two_avg <= CC_GlobalState->brakeTwoMin && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeTwoMin = brake_two_avg;
			}
			if(brake_two_avg >= CC_GlobalState->brakeTwoMax && CC_GlobalState->tractiveActive)
			{
				CC_GlobalState->brakeTwoMax = brake_two_avg;
			}
		}

		/* Map Travel to Pedal Pos */
		brake_travel_one = map(brake_one_avg, CC_GlobalState->brakeOneMin+2, CC_GlobalState->brakeOneMax-5, 0, 100);
		brake_travel_two = map(brake_two_avg, CC_GlobalState->brakeTwoMin+2, CC_GlobalState->brakeTwoMax-5, 0, 100);

		/* Ensure Brake Pots Synced */
		if(brake_travel_one >= brake_travel_two+10 || brake_travel_one <= brake_travel_two-10)
		{
			CC_GlobalState->tractiveActive = false;
		}

		/* Average 2 Brake Travel Positions */
		uint16_t brake_travel = (brake_travel_one+brake_travel_two)/2;

		/* Echo Brake Position */
		if(CC_GlobalState->rollingBrakeValues[0] > 0 && CC_GlobalState->secondaryRollingBrakeValues[0] > 0 && CC_GlobalState->tractiveActive)
		{
			len = sprintf(x, "Data: %li\r\n", brake_travel);
			CC_LogInfo(x, len);
		}
		osSemaphoreRelease(CC_GlobalState->sem);
	}
	return;
}

void state_debug_exit(fsm_t *fsm)
{
	CC_LogInfo("Exit Debugging\r\n", strlen("Exit Debugging\r\n"));
	return;
}

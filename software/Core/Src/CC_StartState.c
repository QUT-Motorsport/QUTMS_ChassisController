state_t startState = { &state_start_enter, &state_start_iterate, &state_start_exit, "Start_s" };

void state_start_enter(fsm_t *fsm) {
	if (CC_GlobalState == NULL) {
		/* Assign memory and nullify Global State */
		CC_GlobalState = malloc(sizeof(CC_GlobalState_t));
		memset(CC_GlobalState, 0, sizeof(CC_GlobalState_t));

		/* As CC_GlobalState is accessible across threads
		 * we need to use a semaphore to access and lock it
		 */
		CC_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);

		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			/* Bind and configure initial global states */

			/* Skip RTD Sequencing Requiring Brake Pressure */
			CC_GlobalState->RTD_Debug = true;

			/* Ignore ADC Errors */
			CC_GlobalState->ADC_Debug = false;

			/* Boards w/ Heartbeats */
			CC_GlobalState->PDM_Debug = true;
			CC_GlobalState->AMS_Debug = true;
			CC_GlobalState->SHDN_1_Debug = true;
			CC_GlobalState->SHDN_2_Debug = true;
			CC_GlobalState->SHDN_3_Debug = true;
			CC_GlobalState->SHDN_IMD_Debug = true;

			/* Inverters */
			CC_GlobalState->Inverter_Debug = true;

			// Fans
			CC_GlobalState->duty_cycle_left_fan = 50;
			CC_GlobalState->duty_cycle_right_fan = 50;

			/* Bound state for system operations */
			CC_GlobalState->tractiveActive = false;
			CC_GlobalState->pdmTrackState = LV_STARTUP | PDM_POWER_CC_MASK;

			CC_GlobalState->shutdown_fault = false;

			/* Allocate CAN Queues */
			CC_GlobalState->CAN1Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			CC_GlobalState->CAN2Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			CC_GlobalState->CAN3Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
			osSemaphoreRelease(CC_GlobalState->sem);
		}

		/* Ensure CANQueue exists */
		if (CC_GlobalState->CAN1Queue == NULL || CC_GlobalState->CAN2Queue == NULL || CC_GlobalState->CAN3Queue == NULL) {
			Error_Handler();
		}
	}

	/* Set initial pin states */
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);

	/* Initiate Startup on PDM */
	PDM_InitiateStartup_t initiateStartup = Compose_PDM_InitiateStartup();
	CAN_TxHeaderTypeDef header = { .ExtId = initiateStartup.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1,
			.TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };
	HAL_CAN_AddTxMessage(&CAN_2, &header, data, &CC_GlobalState->CAN2_TxMailbox);
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
		CC_CAN_Generic_t msg;
		if (osMessageQueueGet(CC_GlobalState->CAN2Queue, &msg, 0U, 0U) == osOK) {
			/* If Startup Ok */
			if (msg.header.ExtId == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0,
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
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(CC_GlobalState->pdmTrackState & (~HV_STARTUP));
		CAN_TxHeaderTypeDef header = { .ExtId = pdmStartup.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
				sizeof(pdmStartup.data), .TransmitGlobalTime = DISABLE, };
		HAL_CAN_AddTxMessage(&hcan2, &header, pdmStartup.data, &CC_GlobalState->CAN2_TxMailbox);

		/* Set Heartbeat Timers */
		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
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

void state_start_exit(fsm_t *fsm) {
	/* All CAN Wake or
	 * Confirmation to Idle
	 * Messages go here over CAN */
	//CC_LogInfo("Exit Start\r\n", strlen("Exit Start\r\n"));
	return;
}

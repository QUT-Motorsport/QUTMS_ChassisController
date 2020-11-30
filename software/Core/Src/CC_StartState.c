#include "can.h"
#include "CC_FSM_States.h"
#include "CC_StartState.h"

CC_Global_State_t *CC_Global_State;
CC_CAN_State_t *CC_CAN_State;
CC_Tractive_State_t *CC_Tractive_State;
CC_Heartbeat_State_t *CC_Heartbeat_State;

CC_main_threads_t *CC_main_threads;

state_t startState = { &state_start_enter, &state_start_iterate, &state_start_exit, "Start_s" };

void state_start_enter(fsm_t *fsm) {
	int len;
	char x[80];

	// assign memory and nullify all global states
	CC_Global_State = malloc(sizeof(CC_Global_State_t));
	CC_Tractive_State = malloc(sizeof(CC_Tractive_State_t));
	CC_Heartbeat_State = malloc(sizeof(CC_Heartbeat_State_t));

	memset(CC_Global_State, 0, sizeof(CC_Global_State_t));
	memset(CC_Tractive_State, 0, sizeof(CC_Tractive_State_t));
	memset(CC_Heartbeat_State, 0, sizeof(CC_Heartbeat_State_t));

	// initialize semaphores for all global states
	CC_Global_State->sem = osSemaphoreNew(3U, 3U, NULL);
	CC_Tractive_State->sem = osSemaphoreNew(3U, 3U, NULL);
	CC_Heartbeat_State->sem = osSemaphoreNew(3U, 3U, NULL);

	CC_CAN_State = malloc(sizeof(CC_CAN_State_t));
	memset(CC_CAN_State, 0, sizeof(CC_CAN_State_t));
	CC_CAN_State->sem = osSemaphoreNew(3U, 3U, NULL);
	CC_CAN_State->CAN1Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
	CC_CAN_State->CAN2Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);
	CC_CAN_State->CAN3Queue = osMessageQueueNew(CC_CAN_QUEUESIZE, sizeof(CC_CAN_Generic_t), NULL);

	// Ensure CANQueue exists
	if (CC_CAN_State->CAN1Queue == NULL || CC_CAN_State->CAN2Queue == NULL || CC_CAN_State->CAN3Queue == NULL) {
		Error_Handler();
	}

	// initialize all startup values

	//if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
	CC_Global_State->ADC_Debug = true;

	// boards with heartbeats
	CC_Global_State->PDM_Debug = false;
	CC_Global_State->AMS_Debug = true;
	CC_Global_State->SHDN_1_Debug = true;
	CC_Global_State->SHDN_2_Debug = true;
	CC_Global_State->SHDN_3_Debug = true;
	CC_Global_State->SHDN_IMD_Debug = true;

	// inverters
	CC_Global_State->Inverter_Debug = true;

	CC_Global_State->pdm_channel_states = LV_STARTUP | PDM_POWER_CC_MASK;

	CC_Global_State->shutdown_fault = false;

	/*osSemaphoreRelease(CC_Global_State->sem);
	 }*/

	//if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
	CC_Tractive_State->accel_min[0] = ACCEL_PEDAL_ONE_MIN;
	CC_Tractive_State->accel_min[1] = ACCEL_PEDAL_TWO_MIN;
	CC_Tractive_State->accel_min[2] = ACCEL_PEDAL_THREE_MIN;

	CC_Tractive_State->accel_max[0] = ACCEL_PEDAL_ONE_MAX;
	CC_Tractive_State->accel_max[1] = ACCEL_PEDAL_TWO_MAX;
	CC_Tractive_State->accel_max[2] = ACCEL_PEDAL_THREE_MAX;

	CC_Tractive_State->brake_min[0] = BRAKE_PEDAL_ONE_MIN;
	CC_Tractive_State->brake_min[1] = BRAKE_PEDAL_TWO_MIN;

	CC_Tractive_State->brake_max[0] = BRAKE_PEDAL_ONE_MAX;
	CC_Tractive_State->brake_max[1] = BRAKE_PEDAL_TWO_MAX;

	CC_Tractive_State->fault_detected = false;
	CC_Tractive_State->tractive_active = false;


	// set initial pin state
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);

	PDM_InitiateStartup_t initiateStartup = Compose_PDM_InitiateStartup();
	CAN_TxHeaderTypeDef header = { .ExtId = initiateStartup.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1,
			.TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };
 	HAL_CAN_AddTxMessage(&hcan2, &header, data, &CC_CAN_State->CAN2_TxMailbox);

 	// small delay to send message
 	HAL_Delay(10);


	// debug tracing
	//CC_LogInfo("Enter Start\r\n", strlen("Enter Start\r\n"));
	return;
}

void state_start_iterate(fsm_t *fsm) {
	int len = 0;
	char x[80];
	/* Skip boot if PDM Debugging Enabled */
	bool boot = CC_Global_State->PDM_Debug;
	uint32_t getPowerChannels = 0;



	// check for PDM boot response
	//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
	while (osMessageQueueGetCount(CC_CAN_State->CAN2Queue) >= 1) {
		CC_CAN_Generic_t msg;

		if (osMessageQueueGet(CC_CAN_State->CAN2Queue, &msg, 0U, 0U) == osOK) {
			sprintf(x, "%d\r\n", msg.header.ExtId);

			if (msg.header.ExtId == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0,
			CAN_TYPE_TRANSMIT, 0x00, 0x0)) {
				// get initial power channels from PDM
				getPowerChannels = 0;
				Parse_PDM_StartupOk(msg.data, &getPowerChannels);

				// start boot sequence
				boot = true;
			}
		}
	}
	/*
	 osSemaphoreRelease(CC_CAN_State->sem);
	 }*/

	if (boot) {
		// set PDM startup channels
		/*if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		 if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {*/
		CC_Global_State->pdm_channel_states &= (~HV_STARTUP);
		PDM_SetChannelStates_t pdmStartup = Compose_PDM_SetChannelStates(CC_Global_State->pdm_channel_states);
		CAN_TxHeaderTypeDef header = { .ExtId = pdmStartup.id, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
				sizeof(pdmStartup.data), .TransmitGlobalTime = DISABLE, };
		HAL_CAN_AddTxMessage(&hcan2, &header, pdmStartup.data, &CC_CAN_State->CAN2_TxMailbox);

		/*osSemaphoreRelease(CC_CAN_State->sem);
		 }
		 osSemaphoreRelease(CC_Global_State->sem);
		 }*/

		/* Set Heartbeat Timers */
		//if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		CC_Heartbeat_State->amsTicks = HAL_GetTick();
		CC_Heartbeat_State->shutdownOneTicks = HAL_GetTick();
		CC_Heartbeat_State->shutdownTwoTicks = HAL_GetTick();
		CC_Heartbeat_State->shutdownThreeTicks = HAL_GetTick();
		CC_Heartbeat_State->shutdownImdTicks = HAL_GetTick();
		CC_Heartbeat_State->inverterTicks = HAL_GetTick();
		/*osSemaphoreRelease(CC_Heartbeat_State->sem);
		 }*/

		/* Engage Idle State (Waiting for RTD) */
		fsm_changeState(fsm, &idleState, "PDM Boot Sequence Initiated");
	}
	return;
}

void state_start_exit(fsm_t *fsm) {

	// start main threads for idle and RTD
	CC_main_threads = malloc(sizeof(CC_main_threads_t));

	// driving_update_pdm
	osThreadAttr_t thread_attributes = { 0 };
	thread_attributes.name = "main_check_heartbeats";
	thread_attributes.priority = (osPriority_t) osPriorityHigh;
	thread_attributes.stack_size = 256;

	CC_main_threads->main_check_heartbeats_handle = osThreadNew(thread_check_heartbeats, NULL, &thread_attributes);

	thread_attributes.name = "main_read_pedals";
	thread_attributes.priority = (osPriority_t) osPriorityHigh;
	thread_attributes.stack_size = 1024;

	CC_main_threads->main_read_pedals_handle = osThreadNew(thread_read_pedals, NULL, &thread_attributes);

	/* All CAN Wake or
	 * Confirmation to Idle
	 * Messages go here over CAN */
	//CC_LogInfo("Exit Start\r\n", strlen("Exit Start\r\n"));
	return;
}

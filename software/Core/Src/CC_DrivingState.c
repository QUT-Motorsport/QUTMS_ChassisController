/*
 * CC_DrivingState.c
 *
 *  Created on: Nov 30, 2020
 *      Author: calvi
 */

#include "CC_FSM_States.h"
#include "CC_DrivingState.h"

const uint16_t inverter_node_ids[NUM_INVERTERS] = { INVERTER_LEFT_NODE_ID,
INVERTER_RIGHT_NODE_ID };

typedef struct CC_driving_threads {
	osThreadId_t driving_update_pdm_handle;
	osThreadId_t driving_update_inverters_handle;
	osThreadId_t driving_read_CAN_handle;
} CC_driving_threads_t;

CC_driving_threads_t *CC_driving_threads;

void thread_driving_update_pdm(void *argument) {

	for (;;) {
		// acquire pedal values
		//if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
			//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
				// update fan lol
				CAN_TxHeaderTypeDef CAN_header;
				CAN_header.IDE = CAN_ID_EXT;
				CAN_header.RTR = CAN_RTR_DATA;
				CAN_header.TransmitGlobalTime = DISABLE;



				// update brake light
				//if (osSemaphoreAcquire(CC_Global_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
					// clear brake light channel
					CC_Global_State->pdm_channel_states &= ~(PDMFLAG_BRAKE_LIGHT);

					// clear rtd siren channel
					CC_Global_State->pdm_channel_states &= ~(PDMFLAG_RTD_SIREN);

					if (CC_Tractive_State->brake_value > 0) {
						// if pedal is pressed at all
						CC_Global_State->pdm_channel_states |= PDMFLAG_BRAKE_LIGHT;
					}

					if ((HAL_GetTick() - CC_Tractive_State->rtd_ticks) < RTD_SIREN_TICKS) {
						// siren is on for first 1.5 seconds of driving
						CC_Global_State->pdm_channel_states |= PDMFLAG_RTD_SIREN;
					}

					PDM_SetChannelStates_t update_pdm_msg = Compose_PDM_SetChannelStates(
							CC_Global_State->pdm_channel_states);
					CAN_header.ExtId = update_pdm_msg.id;
					CAN_header.DLC = sizeof(update_pdm_msg.data);
					HAL_CAN_AddTxMessage(&hcan2, &CAN_header, update_pdm_msg.data, &CC_CAN_State->CAN2_TxMailbox);

					/*osSemaphoreRelease(CC_Global_State->sem);
				}*/

				//osSemaphoreRelease(CC_CAN_State->sem);
			//}

		//	osSemaphoreRelease(CC_Tractive_State->sem);
		//}

		osDelay(200);
	}
	// if we exit loop for some reason
	osThreadTerminate(NULL);
}

void thread_driving_update_inverters(void *argument) {
	int len = 0;
	char x[80];
	CAN_TxHeaderTypeDef inverter_header = { 0 };
	inverter_header.IDE = CAN_ID_STD;
	inverter_header.RTR = CAN_RTR_DATA;
	inverter_header.TransmitGlobalTime = DISABLE;

	CC_SetBool_t inverter_enable = { 0 };
	CC_SetVariable_t inverter_cmd = { 0 };
	uint8_t result = 0;

	for (;;) {

		// send enable command - need to constantly resend since no feedback from inverters
		//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

			for (int i = 0; i < NUM_INVERTERS; i++) {
				inverter_enable = Compose_CC_SetBool(inverter_node_ids[i], 0x01, 0xFFFFFFFF);
				inverter_header.StdId = inverter_enable.id;
				inverter_header.DLC = 8;

				result = HAL_CAN_AddTxMessage(&hcan1, &inverter_header, inverter_enable.data,
						&CC_CAN_State->CAN1_TxMailbox);

			}

			// send accel and brake commands to each inverter
			//if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
				for (int i = 0; i < NUM_INVERTERS; i++) {
					len = sprintf(x, "Inverter: %d Command - A: %d B: %d ", i, CC_Tractive_State->accel_value,
							CC_Tractive_State->brake_value);
					 CC_LogInfo(x, len);

					// set to 0 just in case we're running and tractive has stopped
					uint16_t accel_value = (CC_Tractive_State->tractive_active) ? CC_Tractive_State->accel_value : 0;

					// accel
					inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i], INVERTER_VAR_ACCEL, accel_value);
					inverter_header.StdId = inverter_cmd.id;
					inverter_header.DLC = 8;
					result = HAL_CAN_AddTxMessage(&hcan1, &inverter_header, inverter_cmd.data,
							&CC_CAN_State->CAN1_TxMailbox);
					len = sprintf(x, "res: %d ", result);
										 CC_LogInfo(x, len);
					// brake
					inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i], INVERTER_VAR_BRAKE,
							CC_Tractive_State->brake_value);
					inverter_header.StdId = inverter_cmd.id;
					inverter_header.DLC = 8;
					result = HAL_CAN_AddTxMessage(&hcan1, &inverter_header, inverter_cmd.data,
							&CC_CAN_State->CAN1_TxMailbox);
					len = sprintf(x, "res: %d \r\n", result);
					 CC_LogInfo(x, len);
				}
/*
				osSemaphoreRelease(CC_Tractive_State->sem);
			}
			osSemaphoreRelease(CC_CAN_State->sem);
		}*/

		// update every 20ms
		osDelay(10);
	}

	// if we exit loop for some reason
	osThreadTerminate(NULL);
}

void thread_driving_read_CAN(void *argument) {
	for (;;) {
		//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

			/* Check for Queued CAN Packets on CAN1 */
			while (osMessageQueueGetCount(CC_CAN_State->CAN1Queue) >= 1) {

				char x[80];
				int len;
				CC_CAN_Generic_t msg;
				if (osMessageQueueGet(CC_CAN_State->CAN1Queue, &msg, 0U, 0U) == osOK) {

					if (msg.header.IDE == CAN_ID_STD) {
						/* Inverter Heartbeat */
						if (msg.header.StdId == 0x700 + INVERTER_LEFT_NODE_ID) {

							//if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
								CC_Heartbeat_State->inverterTicks = HAL_GetTick();
								len = sprintf(x, "INVERTER HEARTBEAT\r\n");
								CC_LogInfo(x, len);

								/*osSemaphoreRelease(CC_Heartbeat_State->sem);
							}*/

						}
						/*
						 // Inverter Response Packet
						 else if (msg.header.StdId == 0x580 + INVERTER_LEFT_NODE_ID) {

						 // Motor RPM Response Packet
						 if ((msg.data[2] << 8 | msg.data[1]) == 0x210A) {
						 // Parse Motor RPM
						 int16_t motorRPM = 0;
						 Parse_CC_RequestRPM(msg.data, &motorRPM);

						 // Echo Motor RPM
						 len = sprintf(x, "[%li] Got RPM from CAN1: %i\r\n",
						 (HAL_GetTick() - CC_GlobalState->startupTicks) / 1000, motorRPM);
						 //CC_LogInfo(x, len);
						 } else {
						 // Echo CAN Packet if index not recognised
						 len = sprintf(x, "[%li] Got CAN msg from CAN1: %02lX\r\n",
						 (HAL_GetTick() - CC_GlobalState->startupTicks) / 1000, msg.header.StdId);
						 //CC_LogInfo(x, len);
						 }
						 }
						 */
					}
				}
			}

			// Check for Queued CAN Packets on CAN2
			while (osMessageQueueGetCount(CC_CAN_State->CAN2Queue) >= 1) {
				CC_CAN_Generic_t msg;
				if (osMessageQueueGet(CC_CAN_State->CAN2Queue, &msg, 0U, 0U) == osOK) {
					/* Packet Handler */
					/* AMS Heartbeat */
					if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {

						//if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
							bool initialised;
							bool HVAn;
							bool HVBn;
							bool precharge;
							bool HVAp;
							bool HVBp;
							uint16_t averageVoltage;
							uint16_t runtime;
							Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn, &precharge, &HVAp, &HVBp,
									&averageVoltage, &runtime);
							CC_Heartbeat_State->amsTicks = HAL_GetTick();
							CC_Global_State->AMS_initialized = initialised;
							/*osSemaphoreRelease(CC_Heartbeat_State->sem);
						}*/
					}
					/* Shutdown 1 Heartbeat */
					else if (msg.header.ExtId == Compose_CANId(0x1, 0x06, 0x0, 0x01, 0x01, 0x0)) {
						//if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
							uint8_t segmentState;
							Parse_SHDN_HeartbeatResponse(*((SHDN_HeartbeatResponse_t*) &(msg.data)), &segmentState);
							CC_Heartbeat_State->shutdownOneTicks = HAL_GetTick();
							 /*osSemaphoreRelease(CC_Heartbeat_State->sem);
						}*/
					}
					/* Shutdown IMD Heartbeat */
					else if (msg.header.ExtId == Compose_CANId(0x1, 0x10, 0x0, 0x1, 0x01, 0x0)) {
						//if (osSemaphoreAcquire(CC_Heartbeat_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
							uint8_t pwmState;
							Parse_SHDN_IMD_HeartbeatResponse(*((SHDN_IMD_HeartbeatResponse_t*) &(msg.data)), &pwmState);
							CC_Heartbeat_State->shutdownImdTicks = HAL_GetTick();
							/*osSemaphoreRelease(CC_Heartbeat_State->sem);
						}*/
					}
					/* Shutdown Triggered Fault */
					else if (msg.header.ExtId == Compose_CANId(0x0, 0x06, 0x0, 0x0, 0x0, 0x0)) {
						CC_Global_State->CC_initialized = Send_CC_FatalShutdown("Fatal Shutdown Trigger Fault\r\n",
						true, &CC_CAN_State->CAN1_TxMailbox, &CC_CAN_State->CAN2_TxMailbox,
								&CC_CAN_State->CAN3_TxMailbox, &hcan1, &hcan2, &hcan3, &huart3, INVERTER_LEFT_NODE_ID,
								INVERTER_RIGHT_NODE_ID);

						//osSemaphoreRelease(CC_CAN_State->sem);
						fsm_changeState(fsm, &idleState, "Resetting to Idle to Clean");
					}
				}
			 }
/*
			osSemaphoreRelease(CC_CAN_State->sem);
		}*/

		// update every 20ms
		osDelay(40);
	}

	// if we exit loop for some reason
	osThreadTerminate(NULL);
}

state_t drivingState = { &state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s" };

void state_driving_enter(fsm_t *fsm) {
	int len;
	char x[80];

	HAL_StatusTypeDef result;

	// initialize driving specific variables

	//if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		CC_Tractive_State->fault_detected = false;
		CC_Tractive_State->tractive_active = true;

		// start RTD siren timer
		CC_Tractive_State->rtd_ticks = HAL_GetTick();
/*
		osSemaphoreRelease(CC_Tractive_State->sem);
	}*/

	// turn off RTD button led
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);

	//if (osSemaphoreAcquire(CC_CAN_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {

		// enable inverters
		CAN_TxHeaderTypeDef inverter_header = { 0 };
		inverter_header.IDE = CAN_ID_STD;
		inverter_header.RTR = CAN_RTR_DATA;
		inverter_header.TransmitGlobalTime = DISABLE;

		CC_SetBool_t inverter_enable = { 0 };
		for (int i = 0; i < NUM_INVERTERS; i++) {
			inverter_enable = Compose_CC_SetBool(inverter_node_ids[i], 0x01, 0xFFFFFFFF);
			inverter_header.StdId = inverter_enable.id;
			inverter_header.DLC = 8;

			result = HAL_CAN_AddTxMessage(&hcan1, &inverter_header, inverter_enable.data,
					&CC_CAN_State->CAN1_TxMailbox);

		}
		/*osSemaphoreRelease(CC_CAN_State->sem);
	}*/


		CAN_TxHeaderTypeDef CAN_header;
		CAN_header.IDE = CAN_ID_EXT;
		CAN_header.RTR = CAN_RTR_DATA;
		CAN_header.TransmitGlobalTime = DISABLE;

		// fans on full while driving
		uint16_t fan_duty_cycle = 100;

		len = sprintf(x, "fan: %d\r\n", fan_duty_cycle);
		CC_LogInfo(x, len);

		// tell pdm left fan duty cycle
		PDM_SetDutyCycle_t pdm_set_duty_msg = { 0 };
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_LEFT_FAN, fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
		HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);

		// tell pdm right fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN, fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
		HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);

		// tell pdm back fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN, fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
		HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);

	// start rtos threads for driving state
	CC_driving_threads = malloc(sizeof(CC_driving_threads_t));

	// driving_update_pdm
	osThreadAttr_t thread_attributes = { 0 };
	thread_attributes.name = "driving_update_pdm";
	thread_attributes.priority = (osPriority_t) osPriorityLow;
	thread_attributes.stack_size = 1024;

	 CC_driving_threads->driving_update_pdm_handle = osThreadNew(thread_driving_update_pdm, NULL, &thread_attributes);

	// driving_update_inverters
	thread_attributes.name = "driving_update_inverters";
	thread_attributes.priority = (osPriority_t) osPriorityHigh;
	thread_attributes.stack_size = 1500;

	CC_driving_threads->driving_update_inverters_handle = osThreadNew(thread_driving_update_inverters, NULL,
			&thread_attributes);

	// driving_read_CAN
	thread_attributes.name = "driving_read_CAN";
	thread_attributes.priority = (osPriority_t) osPriorityNormal;
	thread_attributes.stack_size = 1024;

	CC_driving_threads->driving_read_CAN_handle = osThreadNew(thread_driving_read_CAN, NULL, &thread_attributes);

	return;
}

void state_driving_iterate(fsm_t *fsm) {
	int len = 0;
	char x[120];
	// heartbeats is done in thread

	// adc reading is done in thread

	// check throttle and brake implausibilty timeout
	//if (osSemaphoreAcquire(CC_Tractive_State->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		// print pedal positions
		if (false) {
			len = sprintf(x, "b0 %d b1 %d b0 %d b1 %d \r\n", CC_Tractive_State->brake_pedals_raw[0],
					CC_Tractive_State->brake_pedals_raw[1], CC_Tractive_State->brake_pedals[0],
					CC_Tractive_State->brake_pedals[1]);
			CC_LogInfo(x, len);
			len = sprintf(x, "Accel: %d Brake: %d\r\n", CC_Tractive_State->accel_value, CC_Tractive_State->brake_value);
			CC_LogInfo(x, len);
		}

		if (CC_Tractive_State->fault_detected
				&& ((HAL_GetTick() - CC_Tractive_State->implausible_ticks) > IMPLAUSIBILITY_TICK_COUNT)) {
			// engage soft shutdown

			len = sprintf(x, "fault detected\r\n");
			CC_LogInfo(x, len);

			CC_Tractive_State->tractive_active = false;
			CC_LogInfo("Disabling Tractive Operations\r\n", strlen("Disabling Tractive Operations\r\n"));

			CAN_TxHeaderTypeDef inverter_header = { 0 };
			inverter_header.IDE = CAN_ID_STD;
			inverter_header.RTR = CAN_RTR_DATA;
			inverter_header.TransmitGlobalTime = DISABLE;

			CC_SetVariable_t inverter_cmd = { 0 };
			uint8_t result = 0;

			// tell inverters to stop turning
			for (int i = 0; i < NUM_INVERTERS; i++) {
				len = sprintf(x, "Inverter: %d Command - A: %d B: %d\r\n", i, CC_Tractive_State->accel_value,
						CC_Tractive_State->brake_value);
				//CC_LogInfo(x, len);

				// accel
				inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i], INVERTER_VAR_ACCEL, 0);
				inverter_header.StdId = inverter_cmd.id;
				inverter_header.DLC = 8;
				result = HAL_CAN_AddTxMessage(&hcan1, &inverter_header, inverter_cmd.data,
						&CC_CAN_State->CAN1_TxMailbox);
			}

			// Broadcast Soft Shutdown on all CAN lines
			CC_SoftShutdown_t soft_shutdown_msg = Compose_CC_SoftShutdown();
			CAN_TxHeaderTypeDef soft_shutdown_header = { .ExtId = soft_shutdown_msg.id, .IDE = CAN_ID_EXT, .RTR =
			CAN_RTR_DATA, .DLC = 0, .TransmitGlobalTime = DISABLE, };

			HAL_CAN_AddTxMessage(&hcan1, &soft_shutdown_header, NULL, &CC_CAN_State->CAN1_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan2, &soft_shutdown_header, NULL, &CC_CAN_State->CAN2_TxMailbox);
			HAL_CAN_AddTxMessage(&hcan3, &soft_shutdown_header, NULL, &CC_CAN_State->CAN3_TxMailbox);

			// release semaphore since we're exiting
			//osSemaphoreRelease(CC_Tractive_State->sem);

			fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");
			return;

		}

		/*osSemaphoreRelease(CC_Tractive_State->sem);
	}*/

	// motor control - done in thread

	// 50ms
	osDelay(50);

}

void state_driving_exit(fsm_t *fsm) {
	int len;
	char x[80];
	CAN_TxHeaderTypeDef CAN_header;
			CAN_header.IDE = CAN_ID_EXT;
			CAN_header.RTR = CAN_RTR_DATA;
			CAN_header.TransmitGlobalTime = DISABLE;

			// fans on full while driving
			uint16_t fan_duty_cycle = 0;

			len = sprintf(x, "fan: %d\r\n", fan_duty_cycle);
			CC_LogInfo(x, len);

			// tell pdm left fan duty cycle
			PDM_SetDutyCycle_t pdm_set_duty_msg = { 0 };
			pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_LEFT_FAN, fan_duty_cycle);
			CAN_header.ExtId = pdm_set_duty_msg.id;
			CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);

			// tell pdm right fan duty cycle
			pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN, fan_duty_cycle);
			CAN_header.ExtId = pdm_set_duty_msg.id;
			CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);

			// tell pdm back fan duty cycle
			pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN, fan_duty_cycle);
			CAN_header.ExtId = pdm_set_duty_msg.id;
			CAN_header.DLC = sizeof(pdm_set_duty_msg.data);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_header, pdm_set_duty_msg.data, &CC_CAN_State->CAN2_TxMailbox);


	if (CC_driving_threads != NULL) {
		// terminate threads
		osThreadTerminate(CC_driving_threads->driving_read_CAN_handle);
		osThreadTerminate(CC_driving_threads->driving_update_inverters_handle);
		osThreadTerminate(CC_driving_threads->driving_update_pdm_handle);

		// free memory
		free(CC_driving_threads);
		CC_driving_threads = NULL;
	}

	return;
}

#include "CC_FSM_States.h"
#include "main.h"

#define NUM_INVERTERS 2
#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

#define INVERTER_CMD_TICK_COUNT 100
#define INVERTER_ENABLE_TICK_COUNT 15

#define INVERTER_VAR_ACCEL 0x01
#define INVERTER_VAR_BRAKE 0x02

#define POT_DESYNC 250

#define BRAKE_UPDATE_TICK_COUNT 20

extern TIM_HandleTypeDef htim2;

uint16_t inverter_node_ids[NUM_INVERTERS] = { INVERTER_LEFT_NODE_ID,
INVERTER_RIGHT_NODE_ID };

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
#ifndef RTD_DEBUG
		// set duty cycle of left and right, and amu fans
		uint16_t fan_duty_cycle = 100;

		// tell pdm left fan duty cycle
		PDM_SetDutyCycle_t pdm_set_duty_msg = Compose_PDM_SetDutyCycle(
		PDM_PWM_LEFT_FAN, fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);

		// tell pdm right fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN,
				fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);

		// tell pdm amu fan duty cycle
		pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN,
				fan_duty_cycle);
		CAN_header.ExtId = pdm_set_duty_msg.id;
		CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

		CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
				&CC_GlobalState->CAN2_TxMailbox);
#endif
		osSemaphoreRelease(CC_GlobalState->sem);
	}
	/* Start Polling ADC */
	HAL_ADC_Start_DMA(&hadc1, CC_GlobalState->accelAdcValues, 3);
	HAL_ADC_Start_DMA(&hadc2, CC_GlobalState->brakeAdcValues, 2);

	/* Run MicroBasic Script on Inverter */
	CC_SetBool_t runLeftScript = Compose_CC_SetBool(INVERTER_LEFT_NODE_ID, 0x01,
			0xFFFFFFFF);
	CAN_TxHeaderTypeDef runLeftHeader = { .StdId = runLeftScript.id, .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = sizeof(runLeftScript.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&hcan1, &runLeftHeader, runLeftScript.data,
			&CC_GlobalState->CAN1_TxMailbox);

	CC_SetBool_t runRightScript = Compose_CC_SetBool(INVERTER_RIGHT_NODE_ID,
			0x01, 0xFFFFFFFF);
	CAN_TxHeaderTypeDef runRightHeader = { .StdId = runRightScript.id, .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = sizeof(runRightScript.data),
			.TransmitGlobalTime = DISABLE, };
	CC_send_can_msg(&hcan1, &runRightHeader, runRightScript.data,
			&CC_GlobalState->CAN1_TxMailbox);

	// start ADC timer
	HAL_TIM_Base_Start_IT(&htim2);

	// TODO: small delay to wait for a round of ADC DMA to finish???????
	for (int i = 0; i < NUM_BRAKE_SENSORS; i++) {
		initialize_filtering(&CC_GlobalState->pedal_brake[i],
				(uint16_t) CC_GlobalState->brakeAdcValues[i]);
	}

	for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
		initialize_filtering(&CC_GlobalState->pedal_accel[i],
				(uint16_t) CC_GlobalState->accelAdcValues[i]);
	}

	len = sprintf(buf, "exit rtd\r\n");
	CC_LogInfo(buf, len);

	return;
}

void state_driving_iterate(fsm_t *fsm) {
	char x[200];
	uint32_t len;

	if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK) {
		/* Flash RTD */
		if ((HAL_GetTick() - CC_GlobalState->readyToDriveTicks) > 1000) {
			HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
					!CC_GlobalState->rtdLightActive);
			CC_GlobalState->rtdLightActive = !CC_GlobalState->rtdLightActive;
			CC_GlobalState->readyToDriveTicks = HAL_GetTick();

#ifndef RTD_DEBUG

			CC_GlobalState->pdmTrackState = CC_GlobalState->pdmTrackState
					& (~RTD_SIREN_MASK);
			PDM_SetChannelStates_t rtdSiren = Compose_PDM_SetChannelStates(
					CC_GlobalState->pdmTrackState);
			CAN_TxHeaderTypeDef sirenHeader = { .ExtId = rtdSiren.id, .IDE =
			CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(rtdSiren.data),
					.TransmitGlobalTime = DISABLE, };
			CC_send_can_msg(&hcan2, &sirenHeader, rtdSiren.data,
					&CC_GlobalState->CAN2_TxMailbox);

#endif
		}
		/* AMS Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->amsTicks) > 100
				&& !CC_GlobalState->AMS_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown AMS\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&hcan1, &hcan2, &hcan3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownOneTicks) > 100
				&& !CC_GlobalState->SHDN_1_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN1\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&hcan1, &hcan2, &hcan3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Shutdown IMD Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->shutdownImdTicks) > 100
				&& !CC_GlobalState->SHDN_IMD_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown SHDN IMD\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&hcan1, &hcan2, &hcan3, &huart3,
			//					INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
			//			CC_GlobalState->shutdown_fault = true;
		}
		/* Inverter Heartbeat Expiry - Fatal Shutdown */
		if ((HAL_GetTick() - CC_GlobalState->inverterTicks) > 100
				&& !CC_GlobalState->Inverter_Debug) {
			//			CC_GlobalState->ccInit = Send_CC_FatalShutdown("Fatal Shutdown Inverter\r\n", true,
			//					&CC_GlobalState->CAN1_TxMailbox, &CC_GlobalState->CAN2_TxMailbox, &CC_GlobalState->CAN3_TxMailbox,
			//					&hcan1, &hcan2, &hcan3, &huart3,
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
						&CC_GlobalState->CAN3_TxMailbox, &hcan1, &hcan2, &hcan3,
						&huart3,
						INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID);
				CC_GlobalState->shutdown_fault = true;
				fsm_changeState(fsm, &idleState, "Resetting to Idle to Clean");
			}
		}
	}

	// adc reading event - every 5ms - 200hz
	if (CC_GlobalState->adc_reading == 1) {
		CC_GlobalState->adc_reading = 0;

		for (int i = 0; i < NUM_BRAKE_SENSORS; i++) {
			update_filtering(&CC_GlobalState->pedal_brake[i],
					(uint16_t) CC_GlobalState->brakeAdcValues[i]);

			// update max and min values
			if (CC_GlobalState->pedal_brake[i].current_filtered
					> CC_GlobalState->brakeMax[i]) {
				len = sprintf(x, "Updating Brake Max %d to %d from %d\r\n", i,
						CC_GlobalState->pedal_brake[i].current_filtered,
						CC_GlobalState->brakeMax[i]);
				CC_LogInfo(x, len);

				CC_GlobalState->brakeMax[i] =
						CC_GlobalState->pedal_brake[i].current_filtered;
			}

			if (CC_GlobalState->pedal_brake[i].current_filtered
					< CC_GlobalState->brakeMin[i]) {
				len = sprintf(x, "Updating Brake Min %d to %d from %d\r\n", i,
						CC_GlobalState->pedal_brake[i].current_filtered,
						CC_GlobalState->brakeMin[i]);
				CC_LogInfo(x, len);

				CC_GlobalState->brakeMin[i] =
						CC_GlobalState->pedal_brake[i].current_filtered;
			}
		}

		for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
			update_filtering(&CC_GlobalState->pedal_accel[i],
					(uint16_t) CC_GlobalState->accelAdcValues[i]);

			// update max and min values
			if (CC_GlobalState->pedal_accel[i].current_filtered
					> CC_GlobalState->accelMax[i]) {

				len = sprintf(x, "Updating Accel Max %d to %d from %d\r\n", i,
						CC_GlobalState->pedal_accel[i].current_filtered,
						CC_GlobalState->accelMax[i]);
				CC_LogInfo(x, len);

				CC_GlobalState->accelMax[i] =
						CC_GlobalState->pedal_accel[i].current_filtered;
			}

			if (CC_GlobalState->pedal_accel[i].current_filtered
					< CC_GlobalState->accelMin[i]) {
				CC_GlobalState->accelMin[i] =
						CC_GlobalState->pedal_accel[i].current_filtered;

				len = sprintf(x, "Updating Accel Min %d to %d from %d\r\n", i,
						CC_GlobalState->pedal_accel[i].current_filtered,
						CC_GlobalState->accelMin[i]);
				CC_LogInfo(x, len);
			}
		}

		len = sprintf(x, "%d %d %d\r\n",
				CC_GlobalState->pedal_accel[0].current_filtered,
				CC_GlobalState->accelMin[0], CC_GlobalState->accelMax[0]);
		//CC_LogInfo(x, len);
	}

	/*
	 * Read 3 Throttle ADC Values
	 * Read 2 Brake ADC Values
	 */
	uint32_t brake_travel[2];
	uint32_t accel_travel[3];
	uint32_t brake_sum = 0;
	uint32_t accel_sum = 0;

	/* Echo ADC Failure for Debugging */
	if (CC_GlobalState->faultDetected) {
		//CC_LogInfo("ADC Fault Detected\r\n", strlen("ADC Fault Detected\r\n"));
	}
#ifdef NO_DEF
	if (false) {
		if (osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT)
				== osOK) {

			// Calculate Brake Positions with Filter
			for (int i = 0; i < NUM_BRAKE_SENSORS; i++) {
#ifdef DEBUG_RAW_ADC
				len = sprintf(x, "b%d: %d -:%d +:%d \r\n", i,
						CC_GlobalState->brakeAdcValues[i],
						CC_GlobalState->brakeMin[i],
						CC_GlobalState->brakeMax[i]);
				//CC_LogInfo(x, len);
#endif

				// update min and max
				if (CC_GlobalState->brakeAdcValues[i]
						> CC_GlobalState->brakeMax[i]) {
					uint16_t old_val = CC_GlobalState->brakeMax[i];
					CC_GlobalState->brakeMax[i] =
							CC_GlobalState->brakeAdcValues[i];
					len = sprintf(x, "Updating Brake Max %d to %d from %d\r\n",
							i, CC_GlobalState->brakeMax[i], old_val);
					//CC_LogInfo(x, len);
				}

				if (CC_GlobalState->brakeAdcValues[i]
						< CC_GlobalState->brakeMin[i]) {
					uint16_t old_val = CC_GlobalState->brakeMin[i];
					CC_GlobalState->brakeMin[i] =
							CC_GlobalState->brakeAdcValues[i];
					len = sprintf(x, "Updating Brake Min %d to %d from %d\r\n",
							i, CC_GlobalState->brakeMin[i], old_val);
					//CC_LogInfo(x, len);
				}

				CC_GlobalState->rollingBrakeValues[i] =
						CC_GlobalState->brakeAdcValues[i];

				// Map to Max Duty Cycle //
				brake_travel[i] = map(CC_GlobalState->rollingBrakeValues[i],
						CC_GlobalState->brakeMin[i],
						CC_GlobalState->brakeMax[i], 0,
						MAX_DUTY_CYCLE);
			}

			// Calculate Accel Positions with Filter
			for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
#ifdef DEBUG_RAW_ADC
				len = sprintf(x, "a%d: %d -:%d +:%d ", i,
						CC_GlobalState->accelAdcValues[i],
						CC_GlobalState->accelMin[i],
						CC_GlobalState->accelMax[i]);
				//CC_LogInfo(x, len);
#endif

				// update min and max
				if (CC_GlobalState->pedal_accel[i].current_filtered
						> CC_GlobalState->accelMax[i]) {
					uint16_t old_val = CC_GlobalState->accelMax[i];
					CC_GlobalState->accelMax[i] =
							CC_GlobalState->accelAdcValues[i];
					len = sprintf(x, "Updating Accel Max %d to %d from %d\r\n",
							i, CC_GlobalState->accelMax[i], old_val);
					//CC_LogInfo(x, len);
				}

				if (CC_GlobalState->accelAdcValues[i]
						< CC_GlobalState->accelMin[i]) {
					uint16_t old_val = CC_GlobalState->accelMin[i];
					CC_GlobalState->accelMin[i] =
							CC_GlobalState->accelAdcValues[i];
					len = sprintf(x, "Updating Accel Min %d to %d from %d\r\n",
							i, CC_GlobalState->accelMin[i], old_val);
					//CC_LogInfo(x, len);
				}

				// Fetch Value & Apply Filter

				// Map to Max Duty Cycle
				accel_travel[i] = map(
						CC_GlobalState->pedal_accel[i].current_filtered,
						CC_GlobalState->accelMin[i],
						CC_GlobalState->accelMax[i], 0,
						MAX_DUTY_CYCLE);
			}

#ifdef DEBUG_RAW_ADC
			len = sprintf(x, "\r\n");
			//CC_LogInfo(x, len);
#endif

			/* Calculate Faulty ADC Reading */
			bool currentFault = false;
			/*
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

			 // Fault Handling - Comment Out to Stop Faulting
			 if (currentFault) {
			 // New Fault
			 if (!CC_GlobalState->faultDetected) {
			 //				CC_GlobalState->faultDetected = currentFault;
			 //				CC_GlobalState->implausibleTicks = HAL_GetTick();
			 }
			 } else {
			 // Reset Tripped Fault
			 CC_GlobalState->faultDetected = currentFault;
			 CC_GlobalState->implausibleTicks = 0;
			 }
			 */

			// Convert Pedal Positions to Torque Commands
			CC_GlobalState->brakeTravel = MAX_DUTY_CYCLE - brake_travel[0];
			CC_GlobalState->accelTravel = MAX_DUTY_CYCLE - accel_travel[0];

#ifndef DEBUG_RAW_ADC
		if (true) {
			len = sprintf(x, "Accel: %d Brake: %d\r\n",
					CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
			CC_LogInfo(x, len);
		}
#endif
			// apply plausibility check to accelerator for pedal disconnect
			for (int i = 0; i < NUM_ACCEL_SENSORS; i++) {
				// apply plausibilty check for pedal disconnect
				if ((CC_GlobalState->accelAdcValues[i]
						> 1.2 * CC_GlobalState->accelMax[i])
						|| (CC_GlobalState->accelAdcValues[i]
								< 0.8 * CC_GlobalState->accelMin[i])) {
					// basically if any pedal value is outside the acceptable range set pedal value to 0
					CC_GlobalState->accelTravel = 0;
				}
			}



			osSemaphoreRelease(CC_GlobalState->sem);
		}
	}

#endif

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

			result = CC_send_can_msg(&hcan1, &inverter_header,
					inverter_enable.data, &CC_GlobalState->CAN1_TxMailbox);

		}

		CC_GlobalState->inverter_enable_ticks = HAL_GetTick();
	}

	// send accel and brake
	if (((HAL_GetTick() - CC_GlobalState->inverter_cmd_ticks)
			>= INVERTER_CMD_TICK_COUNT) && !CC_GlobalState->faultDetected) {

		// map accelerator value
		CC_GlobalState->accelTravel = MAX_DUTY_CYCLE - map(
				CC_GlobalState->pedal_accel[0].current_filtered,
				CC_GlobalState->accelMin[0], CC_GlobalState->accelMax[0], 0,
				MAX_DUTY_CYCLE);

		CC_GlobalState->brakeTravel = MAX_DUTY_CYCLE - map(
				CC_GlobalState->pedal_brake[0].current_filtered,
				CC_GlobalState->brakeMin[0], CC_GlobalState->brakeMax[0], 0,
				MAX_DUTY_CYCLE);

		// apply dead zones
		if (CC_GlobalState->accelTravel < DEAD_ZONE_ACCEL) {
			CC_GlobalState->accelTravel = 0;
		}
		/*
		if (CC_GlobalState->brakeTravel < DEAD_ZONE_BRAKE) {
			CC_GlobalState->brakeTravel = 0;
		}
*/
		CC_SetVariable_t inverter_cmd = { 0 };

		for (int i = 0; i < NUM_INVERTERS; i++) {
#ifndef DEBUG_RAW_ADC
			len = sprintf(x, "Inverter: %d Command - A: %d B: %d\r\n", i,
					CC_GlobalState->accelTravel, CC_GlobalState->brakeTravel);
			CC_LogInfo(x, len);
#endif
			// accel
			inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i],
			INVERTER_VAR_ACCEL, CC_GlobalState->accelTravel);
			inverter_header.StdId = inverter_cmd.id;
			inverter_header.DLC = 8;
			result = CC_send_can_msg(&hcan1, &inverter_header,
					inverter_cmd.data, &CC_GlobalState->CAN1_TxMailbox);

			// brake
			inverter_cmd = Compose_CC_SetVariable(inverter_node_ids[i],
			INVERTER_VAR_BRAKE, CC_GlobalState->brakeTravel);
			inverter_header.StdId = inverter_cmd.id;
			inverter_header.DLC = 8;
			result = CC_send_can_msg(&hcan1, &inverter_header,
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
//		CC_send_can_msg(&hcan1, &accelZeroLeftHeader, accelZeroLeftCommand.data, &CC_GlobalState->CAN1_TxMailbox);
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
//		CC_send_can_msg(&hcan1, &accelZeroRightHeader, accelZeroRightCommand.data, &CC_GlobalState->CAN1_TxMailbox);
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

	CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

// tell pdm right fan duty cycle
	pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_RIGHT_FAN,
			fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

// tell pdm amu fan duty cycle
	pdm_set_duty_msg = Compose_PDM_SetDutyCycle(PDM_PWM_ACU_FAN,
			fan_duty_cycle);
	CAN_header.ExtId = pdm_set_duty_msg.id;
	CAN_header.DLC = sizeof(pdm_set_duty_msg.data);

	CC_send_can_msg(&hcan2, &CAN_header, pdm_set_duty_msg.data,
			&CC_GlobalState->CAN2_TxMailbox);

	/* Broadcast Soft Shutdown */
	return;
}

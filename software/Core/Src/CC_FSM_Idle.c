/*
 * CC_FSM_Idle.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Calvin
 */
#include <FSM.h>
#include <Timer.h>
#include <queue.h>

#include <QUTMS_can.h>
#include <AMS_CAN_Messages.h>
#include <CC_CAN_Messages.h>

#include "heartbeat.h"
#include "CC_FSM_States.h"
#include "RTD.h"
#include "pedal_adc.h"
#include "can_dict.h"
#include "debugCAN.h"

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit,
		"Idle_s" };

ms_timer_t timer_rtd_light;
void rtd_light_timer_cb(void *args);

void state_idle_enter(fsm_t *fsm) {
	debugCAN_enterState(CC_STATE_ID_Idle);

	// set initial pin states
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
			GPIO_PIN_RESET);

	RTD_state.precharge_done = false;
	RTD_state.precharge_ticks = 0;
	RTD_state.RTD_ticks = 0;

	timer_rtd_light = timer_init(500, true, rtd_light_timer_cb);

	return;
}

void state_idle_iterate(fsm_t *fsm) {
	// check CAN messages

	CAN_MSG_Generic_t msg;

	// CAN2
	while (queue_next(&queue_CAN2, &msg)) {
		if (msg.ID == AMS_HeartbeatResponse_ID) {
			printf("ams\r\n");
			heartbeats.AMS = true;

			bool initialised = false;
			bool HVAn;
			bool HVBn;
			bool precharge;
			bool HVAp;
			bool HVBp;
			uint16_t averageVoltage;
			uint16_t runtime;

			Parse_AMS_HeartbeatResponse(msg.data, &initialised, &HVAn, &HVBn,
					&precharge, &HVAp, &HVBp, &averageVoltage, &runtime);

			if (!RTD_state.AMS_init && initialised) {
				printf("AMS ready\r\n");
			}

			RTD_state.AMS_init = initialised;

		} else if (msg.ID == SHDN_ShutdownTriggered_ID) {
			// send shutdown to inverters
			//inverter_send_shutdown();

			if (msg.data[0] == 0) {
				printf("shutdown\r\n");
			}

			// change to error state
			//fsm_changeState(fsm, &shutdownState, "Fatal Shutdown");
		} else if (msg.ID == AMS_Ready_ID) {
			if (!RTD_state.precharge_done) {
				printf("AMS ready received\r\n");
				RTD_state.precharge_done = true;
			}
		} else if (msg.ID == CC_OBJ_DICT_ID) {
			CC_OD_handleCAN(&msg, &hcan2);
		}
	}

	// CAN1
	while (queue_next(&queue_CAN1, &msg)) {

	}

	// CAN3
	while (queue_next(&queue_CAN3, &msg)) {

	}

#if DEBUG_AMS==1
	RTD_state.AMS_init = true;
	RTD_state.precharge_done = true;
#endif

	//printf("rtd: %i\r\n", HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin));

	if (RTD_state.AMS_init) {
		if (!RTD_state.precharge_enabled) {
			if (!timer_isRunning(&timer_rtd_light)) {
				timer_start(&timer_rtd_light);
			}

			if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin)) {
				if (RTD_state.precharge_ticks == 0) {
					RTD_state.precharge_ticks = HAL_GetTick();

					printf("RTD Pressed\r\n");
				}

				if ((HAL_GetTick() - RTD_state.precharge_ticks)
						> PRECHARGE_BTN_TIME) {
					RTD_state.precharge_enabled = true;
					RTD_state.precharge_done = false;

					// send CAN msg to AMS to start precharge
					AMS_StartUp_t ams_startup = Compose_AMS_StartUp();
					CAN_TxHeaderTypeDef header = { .ExtId = ams_startup.id,
							.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0,
							.TransmitGlobalTime = DISABLE, };
					CC_send_can_msg(&hcan2, &header, NULL);
				}
			} else {
				RTD_state.precharge_ticks = 0;
			}

		} else {
			timer_stop(&timer_rtd_light);

			if (!RTD_state.precharge_done) {
//				// Resend Precharge Command
//				AMS_StartUp_t ams_startup = Compose_AMS_StartUp();
//				CAN_TxHeaderTypeDef header = { .ExtId = ams_startup.id,
//						.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0,
//						.TransmitGlobalTime = DISABLE, };
//				CC_send_can_msg(&hcan2, &header, NULL);

				// wait for AMS to finish precharge
				HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin,
						GPIO_PIN_RESET);
				printf("waiting for precharge\r\n");
			} else if (RTD_state.precharge_done) {
				printf("precharge done\r\n");

				bool brake_pressed = false;
#if RTD_DEBUG == 1
				brake_pressed = true;
#else
				brake_pressed = (current_pedal_values.pedal_brake_mapped
						> BRAKE_PRESSURE_RTD);
#endif

				if (brake_pressed) {
					// light up RTD button
					HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port,
					HSOUT_RTD_LED_Pin, GPIO_PIN_SET);

					if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin)) {
						if (RTD_state.RTD_ticks == 0) {
							RTD_state.RTD_ticks = HAL_GetTick();
						}

						if ((HAL_GetTick() - RTD_state.RTD_ticks)
								>= RTD_BTN_TIME) {
							// Enter Driving State
							fsm_changeState(fsm, &drivingState, "RTD Engaged");
						}
					}
				} else {
					HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port,
					HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
				}
			}

		}
	}

	// update timers
	timer_update(&timer_rtd_light, NULL);

	return;
}

void state_idle_exit(fsm_t *fsm) {
	debugCAN_exitState(CC_STATE_ID_Idle);

	// activate siren

	// broadcast RTD
	CC_ReadyToDrive_t readyToDrive = Compose_CC_ReadyToDrive();
	CAN_TxHeaderTypeDef header = { .ExtId = readyToDrive.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };

	CC_send_can_msg(&hcan1, &header, data);
	CC_send_can_msg(&hcan2, &header, data);
	CC_send_can_msg(&hcan3, &header, data);

	printf("exit idle\r\n");

	return;
}

void rtd_light_timer_cb(void *args) {
	HAL_GPIO_TogglePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin);
}

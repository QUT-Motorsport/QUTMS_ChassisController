/*
 * states_idle.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"
#include "heartbeat.h"
#include "RTD.h"

state_t state_idle = { &state_idle_enter, &state_idle_body, CC_STATE_IDLE };
state_t state_request_pchrg = { &state_request_pchrg_enter, &state_request_pchrg_body, CC_STATE_PRECHARGE_REQUEST };
state_t state_precharge = { &state_precharge_enter, &state_precharge_body, CC_STATE_PRECHARGE };
state_t state_checkInverter = { &state_checkInverter_enter, &state_checkInverter_body, CC_STATE_INVERTER_CHECK };
state_t state_rtdReady = { &state_rtdReady_enter, &state_rtdReady_body, CC_STATE_RTD_RDY };
state_t state_rtdButton = { &state_rtdButton_enter, &state_rtdButton_body, CC_STATE_RTD_BTN };
state_t state_shutdown = { &state_shutdown_enter, &state_shutdown_body, CC_STATE_SHUTDOWN };

ms_timer_t timer_rtd_light;
void rtd_light_timer_cb(void *args);

void state_idle_enter(fsm_t *fsm) {
	// start by turning RTD button on
	RTD_BTN_On();
	timer_rtd_light = timer_init(500, true, rtd_light_timer_cb);

	// start RTD btn toggle timer
	timer_start(&timer_rtd_light);

	RTD_state.precharge_ticks = 0;
}

void state_idle_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	if (AMS_heartbeatState.stateID != AMS_STATE_READY) {
		// something is wrong, AMS has had some fault, probably BMS or shutdown
		// go back to check AMS and wait for AMS to be ready again

		// stop timer
		timer_stop(&timer_rtd_light);

		// turn RTD button off
		RTD_BTN_Off();

		// go back to check AMS
		fsm_changeState(fsm, &state_checkAMS, "AMS not good");
	}
	else {
		if (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin) == GPIO_PIN_SET) {
			// precharge pressed

			if (RTD_state.precharge_ticks == 0) {
				// first detection of press, so start counting
				RTD_state.precharge_ticks = HAL_GetTick();
			}

			if ((HAL_GetTick() - RTD_state.precharge_ticks) > PRECHARGE_BTN_TIME) {
				// precharge held for long enough
				// go into precharge

				// stop timer
				timer_stop(&timer_rtd_light);

				// turn RTD button off
				RTD_BTN_Off();

				// start precharge
				fsm_changeState(fsm, &state_request_pchrg, "Precharge requested");
			}
		}
	}

	// update timers
	timer_update(&timer_rtd_light, NULL);
}

void rtd_light_timer_cb(void *args) {
	// while waiting to start precharge, toggle RTD light every 500ms
	HAL_GPIO_TogglePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin);
}

void state_request_pchrg_enter(fsm_t *fsm) {

}

void state_request_pchrg_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	if ((AMS_heartbeatState.stateID == AMS_STATE_PRECHARGE) || (AMS_heartbeatState.stateID == AMS_STATE_TS_ACTIVE)) {
		// precharge request has been acknowledged and started, or it's already finished so move to precharge to confirm and wait
		fsm_changeState(fsm, &state_precharge, "Precharging");
	}
	else if (AMS_heartbeatState.stateID != AMS_STATE_READY) {
		// if it's in ready, probably just about to start precharge so ignore
		// any other state is an error

		// something has clowned, so go back to AMS health check
		fsm_changeState(fsm, &state_checkAMS, "Precharge error");
	}
}

void state_precharge_enter(fsm_t *fsm) {

}

void state_precharge_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	if (AMS_heartbeatState.stateID == AMS_STATE_TS_ACTIVE) {
		// precharge finished successfully, TS is active
		// go to inverter health check
		fsm_changeState(fsm, &state_checkInverter, "Precharge finished");
	}
	else if ((AMS_heartbeatState.stateID == AMS_STATE_READY) && (AMS_heartbeatState.flags.PCHRG_TIMEOUT == 1)) {
		// precharge timed out
		// go back to idle and start again

		fsm_changeState(fsm, &state_idle, "Precharge timed out");
	}
	else {
		// why tf we go back, something is broken
		// go to AMS health check

		fsm_changeState(fsm, &state_checkAMS, "Precharge error");
	}
}

void state_checkInverter_enter(fsm_t *fsm) {

}

void state_checkInverter_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	bool inverter_good = true;

	for (int i = 0; i < MCISO_COUNT; i++) {
		// check MCISO board is good
		inverter_good = inverter_good && heartbeats.MCISO[i];

		// check connected inverters are good
		inverter_good = inverter_good && (MCISO_heartbeatState[i].errorFlags.HB_INV0 == 1);
		inverter_good = inverter_good && (MCISO_heartbeatState[i].errorFlags.HB_INV1 == 1);
	}

	if (inverter_good) {
		// all MCISO boards and good and all inverters report good on heartbeat so we good
		fsm_changeState(fsm, &state_rtdReady, "Inverters good");
	}
}

void state_rtdReady_enter(fsm_t *fsm) {

}

void state_rtdReady_body(fsm_t *fsm) {

}

void state_rtdButton_enter(fsm_t *fsm) {

}

void state_rtdButton_body(fsm_t *fsm) {

}

void state_shutdown_enter(fsm_t *fsm) {

}

void state_shutdown_body(fsm_t *fsm) {

}

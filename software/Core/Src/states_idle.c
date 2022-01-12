/*
 * states_idle.c
 *
 *  Created on: Dec 21, 2021
 *      Author: Calvin J
 */

#include "states.h"

#include <Timer.h>

#include "heartbeat.h"
#include "RTD.h"
#include "sensor_adc.h"
#include "shutdown.h"

state_t state_idle = { &state_idle_enter, &state_idle_body, CC_STATE_IDLE };
state_t state_request_pchrg = { &state_request_pchrg_enter, &state_request_pchrg_body, CC_STATE_PRECHARGE_REQUEST };
state_t state_precharge = { &state_precharge_enter, &state_precharge_body, CC_STATE_PRECHARGE };
state_t state_checkInverter = { &state_checkInverter_enter, &state_checkInverter_body, CC_STATE_INVERTER_CHECK };
state_t state_rtdReady = { &state_rtdReady_enter, &state_rtdReady_body, CC_STATE_RTD_RDY };
state_t state_rtdButton = { &state_rtdButton_enter, &state_rtdButton_body, CC_STATE_RTD_BTN };

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
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				// stop timer
				timer_stop(&timer_rtd_light);

				// turn RTD button off
				RTD_BTN_Off();

				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		// stop timer
		timer_stop(&timer_rtd_light);

		// turn RTD button off
		RTD_BTN_Off();

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}
#if DEBUG_AMS == 0
	if (AMS_heartbeatState.stateID != AMS_STATE_READY) {
		// something is wrong, AMS has had some fault, probably BMS or shutdown
		// go back to check AMS and wait for AMS to be ready again

		// stop timer
		timer_stop(&timer_rtd_light);

		// turn RTD button off
		RTD_BTN_Off();

		// go back to check AMS
		fsm_changeState(fsm, &state_checkAMS, "AMS not good");
		return;
	}
	else {
#endif
	if (RTD_BTN_Pressed()) {
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
			return;
		}
	}
#if DEBUG_AMS == 0
	}
#endif

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
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

#if DEBUG_AMS == 0
	if ((AMS_heartbeatState.stateID == AMS_STATE_PRECHARGE) || (AMS_heartbeatState.stateID == AMS_STATE_TS_ACTIVE)) {
#endif
	// precharge request has been acknowledged and started, or it's already finished so move to precharge to confirm and wait
	fsm_changeState(fsm, &state_precharge, "Precharging");
	return;
#if DEBUG_AMS == 0
	}
	else if (AMS_heartbeatState.stateID != AMS_STATE_READY) {
		// if it's in ready, probably just about to start precharge so ignore
		// any other state is an error

		// something has clowned, so go back to AMS health check
		fsm_changeState(fsm, &state_checkAMS, "Precharge error");
		return;
	}
#endif
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
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

#if DEBUG_AMS == 0
	if (AMS_heartbeatState.stateID == AMS_STATE_TS_ACTIVE) {
		// precharge finished successfully, TS is active
		// go to inverter health check
		fsm_changeState(fsm, &state_checkInverter, "Precharge finished");
		return;
	}
	else if ((AMS_heartbeatState.stateID == AMS_STATE_READY) && (AMS_heartbeatState.flags.PCHRG_TIMEOUT == 1)) {
		// precharge timed out
		// go back to idle and start again

		fsm_changeState(fsm, &state_idle, "Precharge timed out");
		return;
	}
	else if (AMS_heartbeatState.stateID != AMS_STATE_PRECHARGE){
		// why tf we go back, something is broken
		// go to AMS health check

		fsm_changeState(fsm, &state_checkAMS, "Precharge error");
		return;
	}
#else
	fsm_changeState(fsm, &state_checkInverter, "Precharge finished");
	return;
#endif
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
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

	bool inverter_good = true;

	for (int i = 0; i < MCISO_COUNT; i++) {
		// check MCISO board is good
		inverter_good = inverter_good && heartbeats.MCISO[i];
#if DEBUG_INV == 0
		// check connected inverters are good
		inverter_good = inverter_good && (MCISO_heartbeatState[i].errorFlags.HB_INV0 == 0);
		inverter_good = inverter_good && (MCISO_heartbeatState[i].errorFlags.HB_INV1 == 0);
#endif
	}

	if (inverter_good) {
		// all MCISO boards and good and all inverters report good on heartbeat so we good
		fsm_changeState(fsm, &state_rtdReady, "Inverters good");
		return;
	}
}

void state_rtdReady_enter(fsm_t *fsm) {
	// make sure button is off
	RTD_BTN_Off();
}

void state_rtdReady_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

	bool brake_pressed = false;

#if (RTD_DEBUG == 1) || (BRAKE_NON_CRITICAL == 1)
	brake_pressed = true;
#else
	brake_pressed = (current_sensor_values.pedal_brake_mapped >= sensor_config.brake_min_actuation);
#endif

	if (brake_pressed) {
		fsm_changeState(fsm, &state_rtdButton, "Brakes actuated");
		return;
	}
}

void state_rtdButton_enter(fsm_t *fsm) {
	// brake is actuated so turn RTD button on
	RTD_BTN_On();

	// brakes just got pushed, reset timer
	RTD_state.RTD_ticks = 0;
}

void state_rtdButton_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN1, &msg)) {
		// check for heartbeats
		check_heartbeat_msg(&msg);
	}

	while (queue_next(&queue_CAN2, &msg)) {
		// check for heartbeat
		if (check_heartbeat_msg(&msg)) {
		}
		// check for shutdowns
		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				RTD_BTN_Off();
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	if (!check_bad_heartbeat()) {
		// board has dropped out, go to error state

		RTD_BTN_Off();

		fsm_changeState(fsm, &state_error, "Board died");
		return;
	}

	bool brake_pressed = false;

#if (RTD_DEBUG == 1) || (BRAKE_NON_CRITICAL == 1)
	brake_pressed = true;
#else
	brake_pressed = (current_sensor_values.pedal_brake_mapped >= sensor_config.brake_min_actuation);
#endif

	if (!brake_pressed) {
		fsm_changeState(fsm, &state_rtdReady, "Brakes not actuated");
		return;
	}

	if (RTD_BTN_Pressed()) {
		if (RTD_state.RTD_ticks == 0) {
			// button just pushed
			RTD_state.RTD_ticks = HAL_GetTick();
		}

		uint32_t timeLeft = (HAL_GetTick() - RTD_state.RTD_ticks);

		printf("total: %i, start: %i\r\n", timeLeft, RTD_state.RTD_ticks);

		if (timeLeft > RTD_BTN_TIME) {
			// send RTD message (this is what triggers siren)
			send_RTD();

			// change to driving state
			fsm_changeState(fsm, &state_driving, "RTD Pressed");
			return;
		}
	}
	else {
		// button not pressed, reset timer
		RTD_state.RTD_ticks = 0;
	}
}

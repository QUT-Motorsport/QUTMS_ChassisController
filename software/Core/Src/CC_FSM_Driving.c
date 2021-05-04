/*
 * CC_FSM_Driving.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include <QUTMS_can.h>
#include "CC_FSM_States.h"
#include "heartbeat.h"
#include <FSM.h>
#include <Timer.h>
#include <stdbool.h>

#include "pedal_adc.h"
#include "inverter_roboteq.h"

state_t drivingState = { &state_driving_enter, &state_driving_iterate,
		&state_driving_exit, "Driving_s" };

ms_timer_t timer_inverters;
void inverter_timer_cb(void *args);

int16_t motor_amps[4];

int inv_count = 0;
int enabled_count = 0;

void state_driving_enter(fsm_t *fsm) {
	// send enable to inverters
	roboteq_update_enabled(true);

	// update duty cycle on fans

	// start inverter timer
	timer_inverters = timer_init(5, true, inverter_timer_cb);

	timer_start(&timer_inverters);

	motor_amps[0] = 0;
	motor_amps[1] = 0;
	motor_amps[2] = 0;
	motor_amps[3] = 0;

	inv_count = 0;
	enabled_count = 0;
}

void state_driving_iterate(fsm_t *fsm) {
	// check CAN messages

	CAN_MSG_Generic_t msg;

	// CAN2
	while (queue_next(&queue_CAN2, &msg)) {
		if (msg.ID == AMS_HeartbeatResponse_ID) {
			heartbeats.AMS = true;
		} else if (msg.ID == SHDN_ShutdownTriggered_ID) {
			// send shutdown to inverters
			roboteq_send_shutdown();

			// change to error state
			fsm_changeState(fsm, &shutdownState, "Fatal Shutdown");
		}
	}

	// CAN1
	while (queue_next(&queue_CAN1, &msg)) {
		//printf("CAN1 id:%x\r\n", msg.ID);

		if ((msg.ID & ~0x7F) == 0x580) {
			// response
			uint16_t node_id = msg.ID & 0x7f;
			uint16_t index = msg.data[0] | (msg.data[1] << 8);
			uint8_t subindex = msg.data[2];
			uint16_t raw_data = msg.data[3] | (msg.data[4] << 8);

			uint8_t num_id = node_id & 0b1;

			if (index == 0x2100) {
				// motor amps
				int16_t *temp = (int16_t*) &raw_data;
				motor_amps[num_id * 2 + subindex] = *temp;
			}
		}

		// forward all CAN1 messages to CAN2

		CAN_TxHeaderTypeDef header = { .ExtId = msg.ID, .IDE =
		CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
				.TransmitGlobalTime = DISABLE, };
		CC_send_can_msg(&hcan2, &header, msg.data);

	}

	// CAN3
	while (queue_next(&queue_CAN3, &msg)) {

	}

	// check heartbeats

	// update brake light

	// send pedal values to inverters
	timer_update(&timer_inverters, fsm);
}

void state_driving_exit(fsm_t *fsm) {
	// only exit here for either soft shutdown (pedals out of sync)
	// or big shutdown -> power is going off

	timer_stop(&timer_inverters);
}

void inverter_timer_cb(void *args) {
	// calculate APPS
	if (false) {
		// apps failed

		// turn off inverters nicely
		roboteq_update_enabled(false);

		// disable last stage of rtd so they have to press button again

		// go back to idle
		fsm_changeState((fsm_t*) args, &idleState, "Apps Shutdown");
	}

	enabled_count++;

	if (enabled_count > 10) {
		enabled_count = 0;
		// resend enabled to inverters
		roboteq_update_enabled(true);

	}



	uint16_t accel = current_pedal_values.pedal_accel_mapped[0];

	if (current_pedal_values.pedal_brake_mapped > 100) {
		accel = 0;
	}

	// send pedal values to inverters
	roboteq_send_pedals(accel, current_pedal_values.pedal_brake_mapped);

	inv_count++;

	if (inv_count == 20) {
		inv_count = 0;

		printf("MA: %d %d %d %d\r\n", motor_amps[0], motor_amps[1],
				motor_amps[2], motor_amps[3]);

		roboteq_request_motor_amps();
	}

}

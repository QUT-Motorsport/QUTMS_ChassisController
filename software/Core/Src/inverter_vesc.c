/*
 * inverter_vesc.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include "inverter.h"
#include "inverter_vesc.h"
#include "pedal_adc.h"
#include "can_dict.h"

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

uint16_t vesc_current_max = VESC_CURRENT_MAX;
uint16_t enable_tv = 1;
uint16_t boost = 0;
uint16_t scalar = 0;
uint16_t deadzone = 5;

int32_t vesc_rpm;

bool regen_mode = false;

VESC_SetCurrentBrake_t regenCommand;
VESC_SetCurrent_t torqueCommand;

CAN_TxHeaderTypeDef vescHeader = { .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA };

void vesc_send_shutdown() {
	// Shutdown VESCs
	for (VESC_ID i = FL; i < RR; i++) {
		VESC_Shutdown_t shutdown = Compose_VESC_Shutdown(i);
		CAN_TxHeaderTypeDef header = { .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA,
				.DLC = 0, .ExtId = shutdown.id, };

		CC_send_can_msg(&hcan1, &header, NULL);
	}

	// send shutdown to all lines

	CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
	CAN_TxHeaderTypeDef header = { .ExtId = fatalShutdown.id, .IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };

	CC_send_can_msg(&hcan1, &header, data);
	CC_send_can_msg(&hcan2, &header, data);
	CC_send_can_msg(&hcan3, &header, data);
}

void vesc_update_enabled(bool state) {
	// Not required for the VESCs
	return;
}

void vesc_send_pedals(uint16_t accel, uint16_t brake) {

	bool fast_for_regen = false;

	for (int i = 0; i < 4; i++) {
		if (motor_kmh[i] > 10) {
			fast_for_regen = true;
		}
	}



	bool disable_motor = (current_pedal_values.APPS_disable_motors
			|| current_pedal_values.BSE_disable_motors
			|| current_pedal_values.pedal_disable_motors);

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_INV_CURRENT)) {
		vesc_current_max = OD_getValue(&CC_obj_dict, CC_OD_IDX_INV_CURRENT,
		true);
	}

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_ENABLE_TV)) {
		enable_tv = OD_getValue(&CC_obj_dict, CC_OD_IDX_ENABLE_TV, true);
	}

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_DEADZONE)) {
		deadzone = OD_getValue(&CC_obj_dict, CC_OD_IDX_DEADZONE, true);
	}

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_SCALAR)) {
		scalar = OD_getValue(&CC_obj_dict, CC_OD_IDX_SCALAR, true);
	}

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_BOOST)) {
		boost = OD_getValue(&CC_obj_dict, CC_OD_IDX_BOOST, true);
	}

	// Accel & Brake come in as 0-1000;
	float ac = accel / 1000.0f;
	float br = brake / 1000.0f;

	float torque = 0.0f;
	float regen = 0.0f;

	// only care about calculating torque / regen if we're not pressing on the brake
	// as pedal implausibility will disable torque / regen commands
	if (br <= VESC_BRAKE_THRESHOLD) {
		// Accelerate
		if (ac <= VESC_DEADZONE_MIN) {
			// Regen Zone
			torque = VESC_CURRENT_MIN;
			regen = VESC_REGEN_MAX
					- (VESC_REGEN_MAX * ac * (1.0f / VESC_DEADZONE_MIN));

			if (regen > VESC_REGEN_MAX) {
				regen = VESC_REGEN_MAX;
			} else if (regen < VESC_REGEN_MIN) {
				regen = VESC_REGEN_MIN;
			}

		} else if ((ac > VESC_DEADZONE_MIN) && (ac <= VESC_DEADZONE_MAX)) {
			// Dead Zone
			torque = VESC_CURRENT_MIN;
			regen = VESC_REGEN_MIN;
		} else if (ac > VESC_DEADZONE_MAX) {
			// Torque Zone
			torque = vesc_current_max * (ac - VESC_DEADZONE_MAX)
					* (1.0f / (1.0f - VESC_DEADZONE_MAX));
			regen = VESC_REGEN_MIN;

			if (torque > vesc_current_max) {
				torque = vesc_current_max;
			}
			if (torque < VESC_CURRENT_MIN) {
				torque = VESC_CURRENT_MIN;
			}
		}
	}

	double tvValues[4] = { 1, 1, 1, 1 };
	// calculate torque vectoring

	if (enable_tv == 1) {
		vesc_torque_vectoring(steering_0, &tvValues[0], &tvValues[1],
				&tvValues[2], &tvValues[3]);
	}

	double rearTorque = abs(steering_0) < deadzone ? torque + scalar : torque;
	double torqueRequest[4] = { torque, torque, rearTorque, rearTorque };

	for (VESC_ID i = FL; i <= RR; i++) {
		// If our regen value is > 0, we only send brake command, else send torque command
		if (regen > 0.0f && fast_for_regen) {

			if (!regen_mode) {
				regen_mode = true;

				torqueCommand = Compose_VESC_SetCurrent(i, 0);
				vescHeader.DLC = sizeof(torqueCommand.data);
				vescHeader.ExtId = torqueCommand.id;

				CC_send_can_msg(&hcan1, &vescHeader, torqueCommand.data);
				CC_send_can_msg(&hcan2, &vescHeader, torqueCommand.data);
			}

			// Set Regen
			regenCommand = Compose_VESC_SetCurrentBrake(i, disable_motor ? 0 : regen);
			vescHeader.DLC = sizeof(regenCommand.data);
			vescHeader.ExtId = regenCommand.id;

			CC_send_can_msg(&hcan1, &vescHeader, regenCommand.data);
			CC_send_can_msg(&hcan2, &vescHeader, regenCommand.data);
		} else {
			// Set Torque
			if (regen_mode) {
				regen_mode = false;

				regenCommand = Compose_VESC_SetCurrentBrake(i, 0);
				vescHeader.DLC = sizeof(regenCommand.data);
				vescHeader.ExtId = regenCommand.id;

				CC_send_can_msg(&hcan1, &vescHeader, regenCommand.data);
				CC_send_can_msg(&hcan2, &vescHeader, regenCommand.data);
			}

			torqueCommand = Compose_VESC_SetCurrent(i,
					disable_motor ? 0 : torqueRequest[i] * tvValues[i]);
			vescHeader.DLC = sizeof(torqueCommand.data);
			vescHeader.ExtId = torqueCommand.id;

			CC_send_can_msg(&hcan1, &vescHeader, torqueCommand.data);
			CC_send_can_msg(&hcan2, &vescHeader, torqueCommand.data);
		}
	}

	printf("Demanded Torque, Regen of: [%i, %i](rounded)\r\n", (int) torque,
			(int) regen);
}

void vesc_request_motor_amps() {
	return; // We get this in VESC_Status_1_ID
}

void vesc_torque_vectoring(double steeringAngle, double *fl, double *fr,
		double *rl, double *rr) {
	// l = wheelbase
	// w = trackwidth
	// b = steering angle
	// r = radius of turning circle
	// alpha = angle of centre of rotation to front inner wheel
	// d = distance of  rear inner wheel to centre of rotation
	// rref = radius of turning circle to front outer wheel (primary wheel)

	double b = steeringAngle * M_PI / 180.0f;

	double l = 1.535;
	double w = 1.2;

	double rref;
	double rRL;
	double rRR;
	double rFL;
	double rFR;
	double d;
	double alpha;
	double rboost;
	double lboost;

	if (steeringAngle > deadzone) {
		rFR = l / sin(b);
		d = l / tan(b);
		alpha = atan(l / (w + d));
		rFL = l / sin(alpha);
		rRR = d;
		rRL = d + w;
		rref = rFL;
		lboost = 0;
		rboost = boost;
	} else if (steeringAngle < -(deadzone)) {
		b = b * -1;
		rFL = l / sin(b);
		d = l / tan(b);
		alpha = atan(l / (w + d));
		rFR = l / sin(alpha);
		rRL = d;
		rRR = w + d;
		rref = rFR;
		rboost = 0;
		lboost = boost;
	} else {
		rFR = 1;
		rFL = 1;
		rRR = 1;
		rRL = 1;
		rref = 1;
		boost = 0;
		rboost = 0;
		lboost = 0;
	}

	*fr = (rFR / rref) + rboost;
	*fl = (rFL / rref) + lboost;
	*rr = (rRR / rref) + boost;
	*rl = (rRL / rref) + boost;
}

void vesc_setRPM(int32_t thisRPM) {
	vesc_rpm = thisRPM;
}

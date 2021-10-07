/*
 * inverter_vesc.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include "inverter_vesc.h"
#include "pedal_adc.h"
#include "can_dict.h"

#include <math.h>

bool enable_tv = false;

uint16_t vesc_current_max = VESC_CURRENT_MAX;

int32_t vesc_rpm;

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

	if (OD_flagStatus(&CC_obj_dict, CC_OD_IDX_INV_CURRENT)) {
		vesc_current_max = OD_getValue(&CC_obj_dict, CC_OD_IDX_INV_CURRENT, true);
	}

	// Accel & Brake come in as 0-1000;
	float ac = accel / 1000.0f;
	float br = brake / 1000.0f;

	float torque = 0.0f;
	float regen = 0.0f;

	if (br >= VESC_BRAKE_THRESHOLD) // If we are on the brakes, do not accelerate, full regen
	{
		// Brake
		torque = VESC_CURRENT_MIN;
		regen = VESC_REGEN_MAX;

	} else {
		// Accelerate
		if (ac <= VESC_DEADZONE_MIN) {
			// Regen Zone
			torque = vesc_current_max;
			regen = VESC_REGEN_MAX
					- (VESC_REGEN_MAX * ac * (1.0f / VESC_DEADZONE_MIN));

			if (regen > VESC_REGEN_MAX)
				regen = VESC_REGEN_MAX;
			if (regen < VESC_REGEN_MIN)
				regen = VESC_REGEN_MIN;

		} else if (ac > VESC_DEADZONE_MIN && ac <= VESC_DEADZONE_MAX) {
			// Dead Zone
			torque = VESC_CURRENT_MIN;
			regen = VESC_REGEN_MIN;
		} else if (ac > VESC_DEADZONE_MAX) {
			// Torque Zone
			torque = vesc_current_max * (ac - VESC_DEADZONE_MAX)
					* (1.0f / (1.0f - VESC_DEADZONE_MAX));
			regen = VESC_REGEN_MIN;

			if (torque > vesc_current_max)
				torque = vesc_current_max;
			if (torque < VESC_CURRENT_MIN)
				torque = VESC_CURRENT_MIN;
		}
	}

	double tvValues[4] = { 1, 1, 1, 1 };
	// calculate torque vectoring
/*
	if (enable_tv) {
		vesc_torque_vectoring(steering_0, &tvValues[0], &tvValues[1],
				&tvValues[2], &tvValues[3]);
	}
*/
	for (VESC_ID i = FL; i <= RR; i++) {
		// If our regen value is > 0, we only send brake command, else send torque command
		if (regen > 0.0f && (float)(vesc_rpm / (21.0f * 4.5f)) > 500.0f/4.5f) {
			// Set Regen
			VESC_SetCurrentBrake_t regenCommand = Compose_VESC_SetCurrentBrake(
					i, regen);
			CAN_TxHeaderTypeDef regenHeader = { .IDE = CAN_ID_EXT, .RTR =
					CAN_RTR_DATA, .DLC = sizeof(regenCommand.data), .ExtId =
					regenCommand.id, };

			//CC_send_can_msg(&hcan1, &regenHeader, regenCommand.data);
			//CC_send_can_msg(&hcan2, &regenHeader, regenCommand.data);
		} else {
			// Set Torque
			VESC_SetCurrent_t torqueCommand = Compose_VESC_SetCurrent(i,
					torque);
			CAN_TxHeaderTypeDef torqueHeader = { .IDE = CAN_ID_EXT, .RTR =
					CAN_RTR_DATA, .DLC = sizeof(torqueCommand.data), .ExtId =
					torqueCommand.id, };

			CC_send_can_msg(&hcan1, &torqueHeader, torqueCommand.data);
			CC_send_can_msg(&hcan2, &torqueHeader, torqueCommand.data);
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

	double b = steeringAngle;

	double l = 1.535;
	double w = 1.2;

	double rref;
	double rRL;
	double rRR;
	double rFL;
	double rFR;
	double d;
	double alpha;

	if (b > 0) {
		rFR = l / sin(b);
		d = l / tan(b);
		alpha = atan(l / (w + d));
		rFL = l / sin(alpha);
		rRR = d;
		rRL = d + w;
		rref = rFL;
	} else if (b < 0) {
		b = b * -1;
		rFL = l / sin(b);
		d = l / tan(b);
		alpha = atan(l / (w + d));
		rFR = l / sin(alpha);
		rRL = d;
		rRR = w + d;
		rref = rFR;
	} else {
		rFR = 1;
		rFL = 1;
		rRR = 1;
		rRL = 1;
		rref = 1;
	}

	*fr = (rFR / rref);
	*fl = (rFL / rref);
	*rr = (rRR / rref);
	*rl = (rRL / rref);
}

void vesc_setRPM(int32_t thisRPM)
{
	vesc_rpm = thisRPM;
}

/*
 * inverter.c
 *
 *  Created on: 29 Dec. 2021
 *      Author: Calvin J
 */

#include "inverter.h"

#include <math.h>

#include "sensor_adc.h"
#include "inverter_vesc.h"

inverter_settings_t inverter_config;
int32_t motor_rpm[NUM_MOTORS];
double motor_kmh[NUM_MOTORS];

bool regen_mode = false;

void inverter_setup() {
	inverter_config.max_current = INV_MAX_CURRENT;
	inverter_config.deadzone_min = INV_DEADZONE_MIN;
	inverter_config.deadzone_max = INV_DEADZONE_MAX;

	inverter_config.regen_enable = INV_REGEN_ENABLE;
	inverter_config.regen_kmh_cutoff = INV_REGEN_KMH_CUTOFF;
	inverter_config.regen_max_current = INV_REGEN_MAX_CURRENT;

	inverter_config.TV_enable = INV_TV_ENABLE;
	inverter_config.TV_deadzone = INV_TV_DEADZONE;
	inverter_config.TV_scalar = INV_TV_SCALAR;
	inverter_config.TV_boost = INV_TV_BOOST;

	regen_mode = false;
}

void inverter_shutdown() {
	vesc_send_shutdown();
}

void inverter_send_pedals(uint16_t accel, uint16_t brake, double steeringAngle, bool disable_motor, bool enable_tv) {
	// if motors are disable, send 0 torque and do nothing else
	if (disable_motor) {
		regen_mode = false;
		for (uint8_t i = 0; i <= NUM_MOTORS; i++) {
			inverter_send_torque(i, 0);
		}
		return;
	}

	// motor IDs are as follows

	// FL - 0
	// FR - 1
	// RL - 2
	// RR - 3

	// motors are enabled, so do everything properly

	// are we going fast enough for regen?
	bool fast_for_regen = false;
	for (int i = 0; i < NUM_MOTORS; i++) {
		if (motor_kmh[i] >= inverter_config.regen_kmh_cutoff) {
			fast_for_regen = false;
		}
	}

	float torque = 0.0f;
	float regen = 0.0f;

	// calculate torque and regen currents
	if (brake <= current_sensor_values.brake_min_actuation) {
		if (accel <= inverter_config.deadzone_min) {
			// regen zone

			torque = 0.0f;

			uint16_t regen_flipped = inverter_config.deadzone_min - accel;
			regen = (float) map_capped(regen_flipped, 0, inverter_config.deadzone_min, 0,
					inverter_config.regen_max_current);

		}
		else if ((accel >= inverter_config.deadzone_min) && (accel <= inverter_config.deadzone_max)) {
			// dead zone
		}
		else if (accel > inverter_config.deadzone_max) {
			// torque zone

			torque = (float) map_capped(accel, inverter_config.deadzone_max, PEDAL_DUTY_CYCLE, 0,
					inverter_config.max_current);
			regen = 0.0f;
		}
	}

	// determine if in straight, if so boost rears

	double rearTorque = torque;

	// TODO: I really don't like this from a safety perspective
	// discuss this with @John

	/*
	 if (!current_sensor_values.steering_imp_present) {
	 // no current implausibility, so add scalar to rear if we're within deadzone

	 if (fabs(steeringAngle) < inverter_config.TV_deadzone) {
	 rearTorque = torque + inverter_config.TV_scalar;
	 }
	 }
	 */

	double torqueRequest[4] = { torque, torque, rearTorque, rearTorque };

	// calculate TV if applicable
	double tvValues[4] = { 1, 1, 1, 1 };

	if (enable_tv) {
		inverter_calculate_TV(steeringAngle, tvValues);

	}

	// determine what to send to motors
	if ((regen > 0.0f) && fast_for_regen && (inverter_config.regen_enable == 1)) {
		// send regen command

		if (!regen_mode) {
			regen_mode = true;

			for (uint8_t i = 0; i < NUM_MOTORS; i++) {
				inverter_send_torque(i, 0);
			}
		}

		for (uint8_t i = 0; i < NUM_MOTORS; i++) {
			inverter_send_regen(i, regen);
		}
	}
	else {
		// send torque command

		if (regen_mode && (inverter_config.regen_enable == 1)) {
			regen_mode = false;

			for (uint8_t i = 0; i < NUM_MOTORS; i++) {
				inverter_send_regen(i, 0);
			}
		}

		for (uint8_t i = 0; i < NUM_MOTORS; i++) {
			inverter_send_torque(i, torqueRequest[i] * tvValues[i]);
		}
	}
}

void inverter_calculate_TV(double steeringAngle, double tvValues[4]) {
	// steering angle (radians)
	double sa_r = steeringAngle * M_PI / 180.0;

	double wheelBase = 1.535;
	double trackWidth = 1.2;

	// radius of turning circle to front outer wheel (primary wheel)
	double rRef;

	// radius of turning circle for each wheel
	double rRL;
	double rRR;
	double rFL;
	double rFR;

	// distance of rear inner wheel to centre of rotation
	double dist;

	// angle of centre of rotation to front inner wheel
	double alpha;

	double rBoost;
	double lBoost;

	if (steeringAngle > inverter_config.TV_deadzone) {
		rFR = wheelBase / sin(sa_r);

		dist = wheelBase / tan(sa_r);
		alpha = atan(wheelBase / (dist + trackWidth));

		rFL = wheelBase / sin(alpha);
		rRR = dist;
		rRL = dist + trackWidth;

		rRef = rFL;

		lBoost = 0;
		rBoost = inverter_config.TV_boost;
	} else if (steeringAngle < -inverter_config.TV_deadzone) {
		sa_r = sa_r * -1;

		rFL = wheelBase / sin(sa_r);

		dist = wheelBase / tan(sa_r);
		alpha = atan(wheelBase / (dist + trackWidth) );

		rFR = wheelBase / sin(alpha);
		rRL = dist;
		rRR = dist + trackWidth;

		rRef = rFR;

		lBoost = inverter_config.TV_boost;
		rBoost = 0;
	} else {
		rFR = 1;
		rFL = 1;
		rRR = 1;
		rRL = 1;
		rRef = 1;
		rBoost = 0;
		lBoost = 0;
	}

	// FL
	tvValues[0] = (rFL / rRef) + lBoost;

	// FR
	tvValues[1] = (rFR / rRef) + rBoost;

	// RL
	tvValues[2] = (rRL / rRef) + inverter_config.TV_boost;

	// RR
	tvValues[3] = (rRR / rRef) + inverter_config.TV_boost;
}

void inverter_send_torque(uint8_t id, float request) {
	vesc_send_torque(id, request);
}

void inverter_send_regen(uint8_t id, float request) {
	vesc_send_regen(id, request);
}

void inverter_update_rpm(uint8_t id, int32_t rpm) {
	if (id < NUM_MOTORS) {
		motor_rpm[id] = rpm / (21.0 * 4.50);
		motor_kmh[id] = motor_rpm[id] * 3.14 * WHEEL_RADIUS * 60 / 1000;
	}
}

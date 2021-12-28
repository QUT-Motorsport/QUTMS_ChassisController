/*
 * inverter.h
 *
 *  Created on: 29 Dec. 2021
 *      Author: Calvin J
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include <stdint.h>
#include <stdbool.h>

#define NUM_MOTORS 4

typedef struct inverter_settings {
	uint16_t max_current;

	uint16_t deadzone_min;
	uint16_t deadzone_max;

	uint8_t regen_enable;
	uint16_t regen_kmh_cutoff;
	uint16_t regen_max_current;

	uint8_t TV_enable;
	uint16_t TV_deadzone;
	uint16_t TV_scalar;
	uint16_t TV_boost;

} inverter_settings_t;

extern inverter_settings_t inverter_config;

extern int32_t motor_rpm[NUM_MOTORS];
extern double motor_kmh[NUM_MOTORS];

void inverter_setup();

void inverter_shutdown();

void inverter_send_pedals(uint16_t accel, uint16_t brake, double steeringAngle, bool disable_motor, bool enable_tv);

void inverter_calculate_TV(double steeringAngle, double tvValues[4]);

void inverter_send_torque(uint8_t id, float request);
void inverter_send_regen(uint8_t id, float request);

void inverter_update_rpm(uint8_t id, int32_t rpm);


#endif /* INC_INVERTER_H_ */

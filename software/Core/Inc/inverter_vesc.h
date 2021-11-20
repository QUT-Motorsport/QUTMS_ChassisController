/*
 * inverter_vesc.h
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#ifndef INC_INVERTER_VESC_H_
#define INC_INVERTER_VESC_H_

#define VESC_CURRENT_MAX 120
#define VESC_CURRENT_MIN 0

#define VESC_REGEN_MAX 60
#define VESC_REGEN_MIN 0

#define VESC_BRAKE_THRESHOLD 0.2f
#define VESC_DEADZONE_MIN 0.3f
#define VESC_DEADZONE_MAX 0.4f

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "can.h"
#include "QUTMS_can.h"
#include "CC_CAN_Messages.h"
#include "VESC_CAN_Messages.h"

void vesc_send_shutdown();
void vesc_update_enabled(bool state);
void vesc_send_pedals(uint16_t accel, uint16_t brake);
void vesc_torque_vectoring(double steeringAngle, double* fl, double* fr, double* rl, double* rr);
void vesc_request_motor_amps();
void vesc_setRPM(int32_t rpm);

extern int32_t motor_rpm[4];

#endif /* INC_INVERTER_VESC_H_ */

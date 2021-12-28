/*
 * inverter_vesc.h
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#ifndef INC_INVERTER_VESC_H_
#define INC_INVERTER_VESC_H_

#include <CAN_CC.h>
#include <QUTMS_CAN.h>
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "can.h"
#include "VESC_CAN_Messages.h"

#define TV_ENABLED_DEFAULT 0
#define REGEN_ENABLED_DEFAULT 1

#define VESC_CURRENT_MAX 120
#define VESC_CURRENT_MIN 0

#define VESC_REGEN_KMH_CUTOFF_DEFAULT 10

#define VESC_REGEN_MAX_DEFAULT 60
#define VESC_REGEN_MIN 0

#define VESC_BRAKE_THRESHOLD (BRAKE_MIN_ACTIVATION / 1000.0f)
#define VESC_DEADZONE_MIN 0.3f
#define VESC_DEADZONE_MAX 0.4f


void vesc_send_shutdown();
void vesc_update_enabled(bool state);
void vesc_send_pedals(uint16_t accel, uint16_t brake);
void vesc_torque_vectoring(double steeringAngle, double* fl, double* fr, double* rl, double* rr);
void vesc_request_motor_amps();
void vesc_setRPM(int32_t rpm);

#endif /* INC_INVERTER_VESC_H_ */

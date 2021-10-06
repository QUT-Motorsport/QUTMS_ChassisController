/*
 * inverter.h
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#define INV_VESC

#ifndef INV_VESC
#define INV_ROBOTEQ
#endif

#include "main.h"

#include "inverter_roboteq.h"
#include "inverter_vesc.h"

void inverter_send_shutdown();

void inverter_update_enabled(bool state);

void inverter_send_pedals(uint16_t accel, uint16_t brake);

void inverter_request_motor_amps();

#endif /* INC_INVERTER_H_ */

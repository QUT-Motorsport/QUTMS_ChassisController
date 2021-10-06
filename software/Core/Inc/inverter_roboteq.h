/*
 * inverter_roboteq.h
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#ifndef INC_INVERTER_ROBOTEQ_H_
#define INC_INVERTER_ROBOTEQ_H_

#include <stdbool.h>

#define NUM_INVERTERS 2
#define INVERTER_LEFT_NODE_ID 100
#define INVERTER_RIGHT_NODE_ID 101

#define MOTOR_1_SUBINDEX 0x01
#define MOTOR_2_SUBINDEX 0x02

#define INVERTER_VAR_ACCEL 0x01
#define INVERTER_VAR_BRAKE 0x02

void roboteq_request_motor_amps();

void roboteq_send_shutdown();

void roboteq_update_enabled(bool state);

void roboteq_send_pedals(uint16_t accel, uint16_t brake);

#endif /* INC_INVERTER_ROBOTEQ_H_ */

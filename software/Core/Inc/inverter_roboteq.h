/*
 * inverter_roboteq.h
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#ifndef INC_INVERTER_ROBOTEQ_H_
#define INC_INVERTER_ROBOTEQ_H_

#include <stdbool.h>

void roboteq_send_shutdown();

void roboteq_update_enabled(bool state);

void roboteq_send_pedals(uint16_t accel, uint16_t brake);

#endif /* INC_INVERTER_ROBOTEQ_H_ */

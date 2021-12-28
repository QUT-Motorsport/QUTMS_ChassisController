/*
 * shutdown.h
 *
 *  Created on: 28 Dec. 2021
 *      Author: Calvin
 */

#ifndef INC_SHUTDOWN_H_
#define INC_SHUTDOWN_H_

#include <QUTMS_CAN.h>
#include <stdbool.h>

bool check_shutdown_msg(CAN_MSG_Generic_t *msg, bool *shdn_triggered);

extern bool shutdown_triggered;

#endif /* INC_SHUTDOWN_H_ */

/*
 * data_logger.h
 *
 *  Created on: 5 Dec 2020
 *      Author: Calvin Johnson
 */

#if false

#ifndef INC_DATA_LOGGER_H_
#define INC_DATA_LOGGER_H_

#include "QUTMS_can.h"

typedef struct serial_log {
	uint32_t current_ticks;
	char data[200];
	int len;
} serial_log_t;

typedef struct CAN_log {
	uint32_t current_ticks;
	CAN_MSG_Generic_t can_msg;
} CAN_log_t;

int setup_log_queues();


/**
 * rtos thread to handle all SD card stuff
 */
void thread_data_logger(void *argument);

void add_serial_log(serial_log_t *log_item);
void add_CAN_log(CAN_log_t *log_item);



#endif /* INC_DATA_LOGGER_H_ */
#endif

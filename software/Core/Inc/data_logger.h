/*
 * data_logger.h
 *
 *  Created on: 5 Dec 2020
 *      Author: Calvin Johnson
 */

#ifndef INC_DATA_LOGGER_H_
#define INC_DATA_LOGGER_H_

#include <Timer.h>
#include <queue.h>

#define LOG_SERIAL_QUEUE_SIZE 30
#define LOG_CAN_QUEUE_SIZE 100

typedef struct serial_log {
	uint32_t current_ticks;
	char data[200];
	int len;
} serial_log_t;

extern message_queue_t queue_serial_log;
extern message_queue_t queue_CAN_log;
extern ms_timer_t timer_data_logger;

void setup_data_logger();
void data_logger_timer_cb(void *args);


#if false



#include "QUTMS_can.h"



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



#endif
#endif /* INC_DATA_LOGGER_H_ */

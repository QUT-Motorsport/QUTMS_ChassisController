/*
 * RTD.h
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#ifndef INC_RTD_H_
#define INC_RTD_H_

#include <stdbool.h>
#include <stdint.h>

#define PRECHARGE_BTN_TIME 200
#define RTD_BTN_TIME 5000

// 30% brake to activate RTD
#define BRAKE_PRESSURE_RTD 300

typedef struct RTD {
	bool close_contactors;
	bool precharge_enabled;
	bool precharge_done;
	bool shutdown_fault;
	bool AMS_init;
	uint32_t precharge_ticks;
	uint32_t RTD_ticks;
} RTD_t;

extern RTD_t RTD_state;

void RTD_BTN_Off();
void RTD_BTN_On();

#endif /* INC_RTD_H_ */

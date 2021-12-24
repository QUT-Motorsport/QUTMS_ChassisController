/*
 * can_dict.h
 *
 *  Created on: 7 Oct. 2021
 *      Author: Calvin
 */

#ifndef INC_CAN_DICT_H_
#define INC_CAN_DICT_H_

#include <obj_dic.h>
#include <QUTMS_can.h>
#include <Timer.h>

#include "can.h"

#define CC_OD_IDX_INV_CURRENT 				0x1
#define CC_OD_IDX_ENABLE_TV 				0x2
#define CC_OD_IDX_SCALAR 					0x3
#define CC_OD_IDX_BOOST 					0x4
#define CC_OD_IDX_DEADZONE 					0x5
#define CC_OD_IDX_REGEN_ENABLE 				0x6
#define CC_OD_IDX_REGEN_RPM_CUTOFF			0x7
#define CC_OD_IDX_REGEN_MAX_CURRENT			0x8

extern obj_dict_t CC_obj_dict;
extern ms_timer_t timer_OD;

void CC_OD_init();
void CC_OD_handleCAN(CAN_MSG_Generic_t *msg);
void OD_timer_cb(void *args);

#endif /* INC_CAN_DICT_H_ */

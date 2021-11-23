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
#include "can.h"

#define CC_OD_IDX_INV_CURRENT 0x1
#define CC_OD_IDX_ENABLE_TV 0x2
#define CC_OD_IDX_SCALAR 0x3
#define CC_OD_IDX_BOOST 0x4
#define CC_OD_IDX_DEADZONE 0x5
#define CC_OD_IDX_ENABE_OVERRPM 0x6

extern obj_dict_t CC_obj_dict;

void CC_OD_init();
void CC_OD_handleCAN(CAN_MSG_Generic_t *msg, CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_DICT_H_ */

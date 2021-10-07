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

extern obj_dict_t CC_obj_dict;

void CC_OD_init();
void CC_OD_handleCAN(CAN_MSG_Generic_t *msg, CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_DICT_H_ */

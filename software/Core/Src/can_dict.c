/*
 * can_dict.c
 *
 *  Created on: 7 Oct. 2021
 *      Author: Calvin
 */


#include "can_dict.h"
#include "can.h"

#include "inverter_vesc.h"

obj_dict_t CC_obj_dict;

void CC_OD_init() {
	OD_init(&CC_obj_dict);

	OD_setValue(&CC_obj_dict, CC_OD_IDX_INV_CURRENT, VESC_CURRENT_MAX);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_ENABLE_TV, 0);
}

void CC_OD_handleCAN(CAN_MSG_Generic_t *msg, CAN_HandleTypeDef *hcan) {
	uint8_t outputData[8];

	// interprets CAN message as either get value
	bool sendMsg = OD_handleCAN(&CC_obj_dict, msg->data, outputData);

	if (sendMsg) {
		CAN_TxHeaderTypeDef header = {
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.TransmitGlobalTime = DISABLE
		};

		CC_OBJ_DICT_t msg = Compose_CC_OBJ_DICT(outputData);
		header.ExtId = msg.id;
		header.DLC = sizeof(msg.data);

		CC_send_can_msg(hcan, &header, msg.data);
	}
}

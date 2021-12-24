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
ms_timer_t timer_OD;

void CC_OD_init() {
	OD_init(&CC_obj_dict);

	OD_setValue(&CC_obj_dict, CC_OD_IDX_INV_CURRENT, VESC_CURRENT_MAX);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_DEADZONE, 0);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_SCALAR, 0);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_BOOST, 0);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_ENABLE_TV, TV_ENABLED_DEFAULT);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_REGEN_ENABLE, REGEN_ENABLED_DEFAULT);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_REGEN_RPM_CUTOFF, VESC_REGEN_KMH_CUTOFF_DEFAULT);
	OD_setValue(&CC_obj_dict, CC_OD_IDX_REGEN_MAX_CURRENT, VESC_REGEN_MAX_DEFAULT);

	// object dictionary messages are fairly quiet, so only need to check every 50ms
	timer_OD = timer_init(50, true, OD_timer_cb);

	// start timer
	timer_start(&timer_OD);
}

void CC_OD_handleCAN(CAN_MSG_Generic_t *msg) {
	uint8_t outputData[8];

	// interprets CAN message as either get value
	bool sendMsg = OD_handleCAN(&CC_obj_dict, msg->data, outputData);

	if (sendMsg) {
		CAN_TxHeaderTypeDef header = { .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .TransmitGlobalTime = DISABLE };

		CC_OBJ_DICT_t new_msg = Compose_CC_OBJ_DICT(outputData);
		header.ExtId = new_msg.id;
		header.DLC = sizeof(new_msg.data);

		CC_send_can_msg((CAN_HandleTypeDef *)msg->hcan, &header, new_msg.data);
	}
}

void OD_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN_OD, &msg)) {
		CC_OD_handleCAN(&msg);
	}
}

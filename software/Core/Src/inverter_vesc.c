/*
 * inverter_vesc.c
 *
 *  Created on: 29 Dec. 2021
 *      Author: Calvin
 */

#include "inverter_vesc.h"

#include <CAN_VESC.h>

#include "inverter.h"
#include "can.h"

void vesc_send_shutdown() {
	// Shutdown VESCs
	for (uint8_t i = 0; i < NUM_MOTORS; i++) {
		VESC_Shutdown_t shutdown = Compose_VESC_Shutdown(i);
		CAN_TxHeaderTypeDef header = { .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0, .ExtId = shutdown.id, };

		CC_send_can_msg(&hcan1, &header, NULL);
		CC_send_can_msg(&hcan2, &header, NULL);
	}
}

void vesc_send_torque(uint8_t id, float request) {
	VESC_SetCurrent_t msg = Compose_VESC_SetCurrent(id, request);

	CAN_TxHeaderTypeDef vescHeader;
	vescHeader.IDE = CAN_ID_EXT;
	vescHeader.RTR = CAN_RTR_DATA;
	vescHeader.DLC = sizeof(msg.data);
	vescHeader.ExtId = msg.id;

	CC_send_can_msg(&hcan1, &vescHeader, msg.data);
	CC_send_can_msg(&hcan2, &vescHeader, msg.data);
}

void vesc_send_regen(uint8_t id, float request) {
	VESC_SetCurrentBrake_t msg = Compose_VESC_SetCurrentBrake(id, request);

	CAN_TxHeaderTypeDef vescHeader;
	vescHeader.IDE = CAN_ID_EXT;
	vescHeader.RTR = CAN_RTR_DATA;
	vescHeader.DLC = sizeof(msg.data);
	vescHeader.ExtId = msg.id;

	CC_send_can_msg(&hcan1, &vescHeader, msg.data);
	CC_send_can_msg(&hcan2, &vescHeader, msg.data);
}

bool vesc_handle_CAN(CAN_MSG_Generic_t *msg) {
	bool vesc_msg = false;

	uint32_t msg_type_masked = ((msg->ID & ~0xFF) >> 8);
	uint8_t vesc_id = (msg->ID & 0xFF);

	if (msg_type_masked == VESC_CAN_PACKET_STATUS) {
		vesc_msg = true;

		int32_t rpm;
		float current;
		float duty;
		Parse_VESC_CANPacketStatus(msg->data, &rpm, &current, &duty);

		if (vesc_id < NUM_MOTORS) {
			inverter_update_rpm(vesc_id, rpm);
		}
	}

	return vesc_msg;
}

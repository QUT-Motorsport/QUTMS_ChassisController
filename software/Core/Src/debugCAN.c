/*
 * debugCAN.c
 *
 *  Created on: Oct 12, 2021
 *      Author: Calvin
 */

#include <CAN_Debug.h>
#include "can.h"

void debugCAN_enterState(uint8_t stateID) {
	DEBUG_EnterState_t msg = Compose_DEBUG_EnterState(CAN_SRC_ID_CC, 0, stateID);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };

	CC_send_can_msg(&hcan2, &header, msg.data);
}

void debugCAN_exitState(uint8_t stateID) {
	DEBUG_ExitState_t msg = Compose_DEBUG_ExitState(CAN_SRC_ID_CC, 0, stateID);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };

	CC_send_can_msg(&hcan2, &header, msg.data);
}

void debugCAN_errorPresent(uint16_t errorCode) {
	DEBUG_ErrorPresent_t msg = Compose_DEBUG_ErrorPresent(CAN_SRC_ID_CC, 0, errorCode);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };

	CC_send_can_msg(&hcan2, &header, msg.data);
}

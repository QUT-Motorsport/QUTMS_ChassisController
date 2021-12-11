/*
 * inverter_roboteq.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include <stdint.h>
#include <stdio.h>
#include "can.h"
#include "QUTMS_can.h"
#include "CC_CAN_Messages.h"

#include "inverter_roboteq.h"

void roboteq_request_motor_amps() {
	int subID[2] = { INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID };
	CAN_TxHeaderTypeDef roboteqHeader = { .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8, .TransmitGlobalTime = DISABLE, };

	CC_Roboteq_t readMotorAmps;

	for (int i = 0; i < 2; i++) {
		// motor 1
		readMotorAmps = Compose_Roboteq_CAN(subID[i], 4, 2, 0x2100, 0x01, 0x01);
		roboteqHeader.StdId = readMotorAmps.id;
		CC_send_can_msg(&hcan1, &roboteqHeader, readMotorAmps.data);

		HAL_Delay(1);

		// motor 2
		readMotorAmps = Compose_Roboteq_CAN(subID[i], 4, 2, 0x2100, 0x02, 0x01);
		roboteqHeader.StdId = readMotorAmps.id;
		CC_send_can_msg(&hcan1, &roboteqHeader, readMotorAmps.data);

		HAL_Delay(1);
	}
}

void roboteq_send_shutdown() {
	printf("SENDING SHUTDOWN\r\n");

	int subID[2] = { INVERTER_LEFT_NODE_ID, INVERTER_RIGHT_NODE_ID };
	CAN_TxHeaderTypeDef roboteqHeader = { .IDE =
	CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8, .TransmitGlobalTime = DISABLE, };

	CC_Roboteq_t shutdownInverter;

	for (int i = 0; i < 2; i++) {
		shutdownInverter = Compose_Roboteq_CAN(subID[i], 2, 3, 0x200c, 0x00,
				0x01);
		roboteqHeader.StdId = shutdownInverter.id;
		CC_send_can_msg(&hcan1, &roboteqHeader, shutdownInverter.data);
	}

	// send shutdown to all lines

	CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
	CAN_TxHeaderTypeDef header = { .ExtId = fatalShutdown.id, .IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };
	uint8_t data[1] = { 0xF };
	CC_send_can_msg(&hcan1, &header, data);
	CC_send_can_msg(&hcan2, &header, data);
	CC_send_can_msg(&hcan3, &header, data);
}

void roboteq_update_enabled(bool state) {
	CC_SetBool_t inverter_enable = { 0 };

	uint8_t result = 0;

	CAN_TxHeaderTypeDef inverter_header = { 0 };
	inverter_header.IDE = CAN_ID_STD;
	inverter_header.RTR = CAN_RTR_DATA;
	inverter_header.TransmitGlobalTime = DISABLE;

	inverter_enable = Compose_CC_SetBool(INVERTER_LEFT_NODE_ID, 0x01,
			0xFFFFFFFF);
	inverter_header.StdId = inverter_enable.id;
	inverter_header.DLC = 8;

	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_enable.data);

	inverter_enable = Compose_CC_SetBool(INVERTER_RIGHT_NODE_ID, 0x01,
			0xFFFFFFFF);
	inverter_header.StdId = inverter_enable.id;
	inverter_header.DLC = 8;

	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_enable.data);
}

void roboteq_send_pedals(uint16_t accel, uint16_t brake) {

	CC_SetVariable_t inverter_cmd = { 0 };

	uint8_t result = 0;

	CAN_TxHeaderTypeDef inverter_header = {0};
	inverter_header.IDE = CAN_ID_STD;
	inverter_header.RTR = CAN_RTR_DATA;
	inverter_header.TransmitGlobalTime = DISABLE;

	inverter_cmd = Compose_CC_SetVariable(INVERTER_LEFT_NODE_ID,
	INVERTER_VAR_ACCEL, accel);
	inverter_header.StdId = inverter_cmd.id;
	inverter_header.DLC = 8;
	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_cmd.data);

	inverter_cmd = Compose_CC_SetVariable(INVERTER_RIGHT_NODE_ID,
	INVERTER_VAR_ACCEL, accel);
	inverter_header.StdId = inverter_cmd.id;
	inverter_header.DLC = 8;
	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_cmd.data);

	// brake
	inverter_cmd = Compose_CC_SetVariable(INVERTER_LEFT_NODE_ID,
	INVERTER_VAR_BRAKE, brake);
	inverter_header.StdId = inverter_cmd.id;
	inverter_header.DLC = 8;
	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_cmd.data);

	inverter_cmd = Compose_CC_SetVariable(INVERTER_RIGHT_NODE_ID,
	INVERTER_VAR_BRAKE, brake);
	inverter_header.StdId = inverter_cmd.id;
	inverter_header.DLC = 8;
	result = CC_send_can_msg(&hcan1, &inverter_header, inverter_cmd.data);

	printf("A: %i B: %i\r\n", accel, brake);
}

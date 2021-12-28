/*
 * RTD.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "RTD.h"
#include "main.h"
#include "can.h"

#include <CAN_CC.h>

RTD_t RTD_state;

void RTD_BTN_Off() {
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
}

void RTD_BTN_On() {
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);
}

bool RTD_BTN_Pressed() {
	return (HAL_GPIO_ReadPin(RTD_INPUT_GPIO_Port, RTD_INPUT_Pin) == GPIO_PIN_SET);
}

void send_RTD() {
	CC_RTD_t msg = Compose_CC_RTD();
	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0, .TransmitGlobalTime = DISABLE };

	// send heartbeat on all CAN lines
	CC_send_can_msg(&hcan1, &header, NULL);
	CC_send_can_msg(&hcan2, &header, NULL);
	CC_send_can_msg(&hcan3, &header, NULL);
}

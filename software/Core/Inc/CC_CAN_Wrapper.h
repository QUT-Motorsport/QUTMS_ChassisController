/**
 ******************************************************************************
 * @file CC_CAN_Wrapper.h
 * @brief Chassis Controller CAN Wrapper
 ******************************************************************************
 */

#ifndef INC_CC_CAN_WRAPPER_H_
#define INC_CC_CAN_WRAPPER_H_

#include "stdbool.h"
#include "main.h"
#include "can.h"
#include <stdlib.h>
#include "QUTMS_can.h"
#include "usart.h"
#include "string.h"
#include "CC_CAN_Messages.h"

/**
 * @brief Echo & Send Chassis Controller Fatal Shutdown
 * @param errorCause Source of Error
 * @param echo Echo errorCause over USART3
 * @return void
 */
bool Send_CC_FatalShutdown(char* errorCause, bool echo,
		uint32_t* CAN1_Mailbox, uint32_t* CAN2_Mailbox, uint32_t* CAN3_Mailbox,
		CAN_HandleTypeDef* CanHandle, CAN_HandleTypeDef* CanHandle2, CAN_HandleTypeDef* CanHandle3,
		UART_HandleTypeDef* huartHandle, uint8_t INVERTER_1_NODE_ID, uint8_t INVERTER_2_NODE_ID);

#endif /* INC_CC_CAN_WRAPPER_H_ */

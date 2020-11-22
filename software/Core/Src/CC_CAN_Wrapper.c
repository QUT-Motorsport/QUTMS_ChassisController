/**
 ******************************************************************************
 * @file CC_CAN_Wrapper.c
 * @brief Chassis Controller CAN Wrapper
 ******************************************************************************
 */

#include "CC_CAN_Wrapper.h"

bool Send_CC_FatalShutdown(char* errorCause, bool echo,
		uint32_t* CAN1_Mailbox, uint32_t* CAN2_Mailbox, uint32_t* CAN3_Mailbox,
		CAN_HandleTypeDef* CanHandle, CAN_HandleTypeDef* CanHandle2, CAN_HandleTypeDef* CanHandle3,
		UART_HandleTypeDef* huartHandle, uint8_t INVERTER_1_NODE_ID, uint8_t INVERTER_2_NODE_ID)
{
	if(echo)
	{
		HAL_UART_Transmit(huartHandle, (uint8_t *)errorCause, (size_t)strlen(errorCause), HAL_MAX_DELAY);
	}
	CC_ShutdownInverter_t shutdownInverter = Compose_CC_ShutdownInverter(INVERTER_1_NODE_ID);
	CAN_TxHeaderTypeDef shutdownheader =
	{
			.StdId = shutdownInverter.id,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = 8,
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(CanHandle, &shutdownheader, shutdownInverter.data, CAN1_Mailbox);

	CC_ShutdownInverter_t shutdownSecondInverter = Compose_CC_ShutdownInverter(INVERTER_2_NODE_ID);
	CAN_TxHeaderTypeDef shutdownSecondheader =
	{
			.StdId = shutdownSecondInverter.id,
			.IDE = CAN_ID_STD,
			.RTR = CAN_RTR_DATA,
			.DLC = 8,
			.TransmitGlobalTime = DISABLE,
	};
	HAL_CAN_AddTxMessage(CanHandle, &shutdownSecondheader, shutdownSecondInverter.data, CAN1_Mailbox);

	CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = fatalShutdown.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};
	uint8_t data[1] = {0xF};
	HAL_CAN_AddTxMessage(CanHandle, &header, data, CAN1_Mailbox);
	HAL_CAN_AddTxMessage(CanHandle2, &header, data, CAN2_Mailbox);
	HAL_CAN_AddTxMessage(CanHandle3, &header, data, CAN3_Mailbox);

	return true;
}


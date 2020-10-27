/**
 ******************************************************************************
 * @file CC_CAN_Messages.h
 * @brief Chassis Controller CAN Messages
 ******************************************************************************
 */

#ifndef INC_CC_CAN_MESSAGES_H_
#define INC_CC_CAN_MESSAGES_H_

#include "stdbool.h"
#include "main.h"
#include "can.h"
#include <stdlib.h>
#include "QUTMS_can.h"

/**
 * @brief Chassis Controller RTD Message
 */
typedef struct
{
	uint32_t id; /**< CAN Packet ID */
} CC_ReadyToDrive_t;

/**
 * @brief Chassis Controller RTD Message Composer
 * @param void
 * @return The composed CC_ReadyToDrive_t packet
 */
CC_ReadyToDrive_t Compose_CC_ReadyToDrive(void);

/**
 * @brief Chassis Controller Fatal Shutdown Message
 */
typedef struct
{
	uint32_t id; /**< CAN Packet ID */
} CC_FatalShutdown_t;

/**
 * @brief Chassis Controller Fatal Shutdown Message Composer
 * @param void
 * @return The composed CC_FatalShutdown_t packet
 */
CC_FatalShutdown_t Compose_CC_FatalShutdown(void);

/**
 * @brief Chassis Controller Fatal Shutdown Message
 */
typedef struct
{
	uint32_t id; /**< CAN Packet ID */
} CC_SoftShutdown_t;

/**
 * @brief Chassis Controller Soft Shutdown Message Composer
 * @param void
 * @return The composed CC_SoftShutdown_t packet
 */
CC_SoftShutdown_t Compose_CC_SoftShutdown(void);

#endif /* INC_CC_CAN_MESSAGES_H_ */

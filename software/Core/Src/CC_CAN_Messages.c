/**
 ******************************************************************************
 * @file CC_CAN_Messages.c
 * @brief Chassis Controller CAN Messages
 ******************************************************************************
 */

#include "CC_CAN_Messages.h"

CC_ReadyToDrive_t Compose_CC_ReadyToDrive(void)
{
	CC_ReadyToDrive_t p;
	p.id = Compose_CANId(0x2, 0x16, 0x0, 0x0, 0x0, 0x0);
	return p;
}

CC_FatalShutdown_t Compose_CC_FatalShutdown(void)
{
	CC_FatalShutdown_t p;
	p.id = Compose_CANId(0x2, 0x17, 0x0, 0x0, 0x0, 0x0);
	return p;
}

CC_SoftShutdown_t Compose_CC_SoftShutdown(void)
{
	CC_SoftShutdown_t p;
	p.id = Compose_CANId(0x2, 0x18, 0x0, 0x0, 0x0, 0x0);
	return p;
}

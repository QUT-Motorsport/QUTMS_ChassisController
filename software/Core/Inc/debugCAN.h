/*
 * debugCAN.h
 *
 *  Created on: Oct 12, 2021
 *      Author: Calvin
 */

#ifndef INC_DEBUGCAN_H_
#define INC_DEBUGCAN_H_

#include <DEBUG_CAN_Messages.h>

void debugCAN_enterState(uint8_t stateID);
void debugCAN_exitState(uint8_t stateID);
void debugCAN_errorPresent(uint16_t errorCode);

#endif /* INC_DEBUGCAN_H_ */

/*
 * RTD.c
 *
 *  Created on: Apr 26, 2021
 *      Author: Calvin
 */

#include "RTD.h"
#include "main.h"

RTD_t RTD_state;

void RTD_BTN_Off() {
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_RESET);
}

void RTD_BTN_On() {
	HAL_GPIO_WritePin(HSOUT_RTD_LED_GPIO_Port, HSOUT_RTD_LED_Pin, GPIO_PIN_SET);
}

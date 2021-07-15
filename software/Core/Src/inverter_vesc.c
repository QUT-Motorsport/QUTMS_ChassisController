/*
 * inverter_vesc.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include "inverter_vesc.h"

void vesc_send_shutdown()
{
	// Shutdown VESCs
	for(VESC_ID i = FL; i < RR; i++)
	{
		VESC_Shutdown_t shutdown = Compose_VESC_Shutdown(i);
		CAN_TxHeaderTypeDef header = {
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = 0,
				.ExtId = shutdown.id,
		};

		CC_send_can_msg(&hcan1, &header, NULL);
	}

	// send shutdown to all lines

	CC_FatalShutdown_t fatalShutdown = Compose_CC_FatalShutdown();
	CAN_TxHeaderTypeDef header = {
			.ExtId = fatalShutdown.id,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1, .TransmitGlobalTime = DISABLE,
	};
	uint8_t data[1] = { 0xF };

	CC_send_can_msg(&hcan1, &header, data);
	CC_send_can_msg(&hcan2, &header, data);
	CC_send_can_msg(&hcan3, &header, data);
}

void vesc_update_enabled(bool state)
{
	// Not required for the VESCs
	return;
}

void vesc_send_pedals(uint16_t accel, uint16_t brake)
{
	// Accel & Brake come in as 0-1000;
	float ac = accel / 1000.0f;
	float br = brake / 1000.0f;

	float torque = 0.0f;
	float regen = 0.0f;

	if(br >= VESC_BRAKE_THRESHOLD) // If we are on the brakes, do not accelerate, full regen
	{
		// Brake
		torque = VESC_CURRENT_MIN;
		regen = VESC_REGEN_MAX;

	} else
	{
		// Accelerate
		if(ac <= VESC_DEADZONE_MIN)
		{
			// Regen Zone
			torque = VESC_CURRENT_MIN;
			regen = VESC_REGEN_MAX - (VESC_REGEN_MAX * ac * (1.0f/VESC_DEADZONE_MIN));

			if(regen > VESC_REGEN_MAX) regen = VESC_REGEN_MAX;
			if(regen < VESC_REGEN_MIN) regen = VESC_REGEN_MIN;

		} else if(ac > VESC_DEADZONE_MIN && ac <= VESC_DEADZONE_MAX)
		{
			// Dead Zone
			torque = VESC_CURRENT_MIN;
			regen = VESC_REGEN_MIN;
		} else if(ac > VESC_DEADZONE_MAX)
		{
			// Torque Zone
			torque = VESC_CURRENT_MAX * (ac - VESC_DEADZONE_MAX) * (1.0f/(1.0f - VESC_DEADZONE_MAX));
			regen = VESC_REGEN_MIN;

			if(torque > VESC_CURRENT_MAX) torque = VESC_CURRENT_MAX;
			if(torque < VESC_CURRENT_MIN) torque = VESC_CURRENT_MIN;
		}
	}

	for(VESC_ID i = FL; i <= RR; i++)
	{
		// Set Torque
		VESC_SetCurrent_t torqueCommand = Compose_VESC_SetCurrent(i, torque);
		CAN_TxHeaderTypeDef torqueHeader = {
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(torqueCommand.data),
				.ExtId = torqueCommand.id,
		};

		CC_send_can_msg(&hcan1, &torqueHeader, torqueCommand.data);
		CC_send_can_msg(&hcan2, &torqueHeader, torqueCommand.data);

		// Set Regen
		VESC_SetCurrentBrake_t regenCommand = Compose_VESC_SetCurrentBrake(i, regen);
		CAN_TxHeaderTypeDef regenHeader = {
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(regenCommand.data),
				.ExtId = regenCommand.id,
		};

		//CC_send_can_msg(&hcan1, &regenHeader, regenCommand.data);
		//CC_send_can_msg(&hcan2, &regenHeader, regenCommand.data);
	}

	printf("Demanded Torque, Regen of: [%i, %i](rounded)\r\n", (int)torque, (int)regen);
}

void vesc_request_motor_amps()
{
	return; // We get this in VESC_Status_1_ID
}

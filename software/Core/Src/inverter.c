/*
 * inverter.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include "inverter.h"

void inverter_send_shutdown()
{
#ifdef INV_ROBOTEQ
	roboteq_send_shutdown();
#endif

#ifdef INV_VESC
	vesc_send_shutdown();
#endif
}

void inverter_update_enabled(bool state)
{
#ifdef INV_ROBOTEQ
	roboteq_update_enabled(state);
#endif

#ifdef INV_VESC
	vesc_update_enabled(state);
#endif
}

void inverter_send_pedals(uint16_t accel, uint16_t brake)
{
#ifdef INV_ROBOTEQ
	roboteq_send_pedals(accel, brake);
#endif

#ifdef INV_VESC
	vesc_send_pedals(accel, brake);
#endif
}

void inverter_request_motor_amps()
{
#ifdef INV_ROBOTEQ
	roboteq_request_motor_amps();
#endif

#ifdef INV_VESC
	vesc_request_motor_amps();
#endif
}

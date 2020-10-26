/**
 ******************************************************************************
 * @file CC_FSM_States.c
 * @brief Chassis Controller FSM States
 ******************************************************************************
 */

#include <CC_FSM_States.h>

state_t deadState = {&state_dead_enter, &state_dead_iterate, &state_dead_exit, "Dead_s"};

void state_dead_enter(fsm_t *fsm)
{
	return;
}

void state_dead_iterate(fsm_t *fsm)
{
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm)
{
	return;
}

state_t idleState = {&state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s"};

void state_idle_enter(fsm_t *fsm)
{
	if(CC_GlobalState == NULL)
	{
		CC_GlobalState = malloc(sizeof(CC_GlobalState_t));
		memset(CC_GlobalState, 0, sizeof(CC_GlobalState_t));

		// As CC_GlobalState is accessible across threads, we need to use a semaphore to access it
		CC_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);
		if(osSemaphoreAcquire(CC_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			// TODO Bind and configure initial global states

			osSemaphoreRelease(CC_GlobalState->sem);
		}
	}

	/* Set initial pin states */

	/* Initiate Startup on PDM */

	/*
	 * Select Channels for Startup on PDM
	 * Ensure Channels are not already enabled? (May not be needed)
	 */

	/* Within 100 Seconds, ensure Heartbeats are ok
	 * and shutdown loop closed
	 */
}

void state_idle_iterate(fsm_t *fsm)
{
	//	while(osMessageQueueGetCount(CC_GlobalState->CANQueue) >= 1)
	//	{
	//		CC_CAN_Generic_t msg;
	//		if(osMessageQueueGet(CC_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
	//		{
	//			/* Packet Handler */
	//		}
	//	}

	// TODO Implementation

	/* If Brake Pressure > 20% */

		/* Illuminate Power Button */

		/* If RTD Button Engaged */

			/* Enter Driving State */
			//fsm_changeState(fsm, &drivingState, "RTD Engaged");
}

void state_idle_exit(fsm_t *fsm)
{
	/* Broadcast RTD on all CAN lines */

	return;
}

state_t drivingState = {&state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{
	/* If AMS Contactors Closed & BMS' Healthy */

		/* Play RTD Siren for 2 Seconds */

		/* Enable all channels on PDM */

	/* Else */

		/* Hard Shutdown Power Off */
	return;
}


void state_driving_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(CC_GlobalState->CANQueue) >= 1)
	{
		CC_CAN_Generic_t msg;
		if(osMessageQueueGet(CC_GlobalState->CANQueue, &msg, 0U, 0U))
		{
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 *
			 * Heartbeats:
			 * AMS_Heartbeat, PDM_Heartbeat, SHDN_Heartbeat
			 * BSPD_Heartbeat, IMD_Heartbeat
			 *
			 * Steering Wheel:
			 * STRW_Adjust
			 *
			 * State of Charge:
			 * AMS_SoC
			 *
			 * Error/Warning Codes:
			 * None
			 */
			/* Soft Shutdown Requested */
			//fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");
		}
	}

	/*
	 * Read 3 Throttle ADC Values
	 * Read 2 Brake ADC Values
	 */

	/*
	 * Calculate Throttle Implausibility
	 * Still implausible with only 2 of the 3 ADC values?
	 * Calculate Brake Implausibility
	 */

	/*
	 * Average Throttle Values to Position
	 * Average Brake Values to Position
	 */

	/*
	 * Log Throttle Position
	 * Log Brake Position
	 */

	/*
	 * Read Steering Angle ADC Value
	 * Log Steering Angle
	 */

	/* If Throttle and Brake Implausibility State Clock < 100ms */

		/*
		 * Call Torque Vectoring Algorithm
		 */

		/*
		 * Calculate Regen
		 */

		/*
		 * Send Desired Accel to Inverters
		 */

	/*
	 * If Throttle or Brake Implausibility State Clock > 1000ms
	 * Engage Soft Shutdown (Reset to Idle)
	 */
	//fsm_changeState(fsm, &idleState, "Soft Shutdown Requested (CAN)");

	/*
	 * If 500ms has exceeded since SoC Request
	 * Request State of Charge
	 */
}

void state_driving_exit(fsm_t *fsm)
{
	/* Broadcast Soft Shutdown */
	return;
}

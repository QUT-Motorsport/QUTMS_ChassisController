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

	/* Select Channels for Startup on PDM */

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
}

void state_idle_exit(fsm_t *fsm)
{
  /* Broadcast RTD on all CAN lines */

  return;
}

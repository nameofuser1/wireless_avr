/*
 * SoftwareTimer.c
 *
 *  Created on: 02 дек. 2015 г.
 *      Author: kripton
 */

#include "SoftwareTimer.h"

/*
 * **********************************
 * Init new timer
 * **********************************
 */
void SoftwareTimer_Init(SoftwareTimer *tim)
{
	tim->cb = NULL;
	tim->state = Timer_Idle;
	tim->ticks = 0;
	tim->type = Timer_OnePulse;
	tim->length = 0;
}


/*
 * *****************************************
 * Set callback. Callback is called every
 * time timer tick event occurs
 * *****************************************
 */
void SoftwareTimer_Add_cb(SoftwareTimer *tim, SoftTimerCallback cb)
{
	tim->cb = cb;
}


/*
 * *****************************************
 * Preparing timer for start
 * *****************************************
 */
void SoftwareTimer_Arm(SoftwareTimer *tim, SoftTimerType type, uint32_t length)
{
	tim->state = Timer_Idle;
    tim->type = type;
	tim->length = length;
	tim->ticks = length;
}


/*
 * *******************************************
 * !!! IMPORTANT !!!
 * Can wait only for one pulse timers
 * For repeating timers please use callback
 * *******************************************
 */
void SoftwareTimer_WaitFor(SoftwareTimer *tim)
{
	if(tim->state != Timer_Active)
	{
		return;
	}

	if(tim->type == Timer_OnePulse)
	{
		while(tim->state != Timer_Done);
	}
}




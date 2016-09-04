/*
 * SoftwareTimer.h
 *
 *  Created on: 01 дек. 2015 г.
 *      Author: kripton
 */

#ifndef SOFT_TIMERS_SOFTWARETIMER_H_
#define SOFT_TIMERS_SOFTWARETIMER_H_

#include "common/LinkedList.h"

typedef  void (*SoftTimerCallback)(void);


typedef enum {
	Timer_Active,
	Timer_Idle,
	Timer_Done
} SoftTimerState;

typedef enum {
	Timer_Repeat,
	Timer_OnePulse
} SoftTimerType;

typedef struct {
	SoftTimerState 		state;
	SoftTimerType 		type;
	SoftTimerCallback 	cb;
	uint32_t		    ticks;
	uint32_t	    	length;
}	SoftwareTimer;


void SoftwareTimer_Init(SoftwareTimer *tim);
void SoftwareTimer_Add_cb(SoftwareTimer *tim, SoftTimerCallback cb);
void SoftwareTimer_Arm(SoftwareTimer *tim, SoftTimerType type, uint32_t length);
void SoftwareTimer_WaitFor(SoftwareTimer *tim);



#endif /* SOFT_TIMERS_SOFTWARETIMER_H_ */

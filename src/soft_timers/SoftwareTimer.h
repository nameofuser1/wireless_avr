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
typedef  LinkedList SoftTimerList;

typedef enum {
	Active,
	Idle,
	Done
} SoftTimerState;

typedef enum {
	Repeat,
	OnePulse
} SoftTimerType;

typedef struct {

	SoftTimerState 		state;
	SoftTimerType 		type;
	SoftTimerCallback 	cb;
	uint32_t		    ticks;
	uint32_t	    	length;
}	SoftwareTimer;


void SoftwareTimer_init(SoftwareTimer *tim);
void SoftwareTimer_add_cb(SoftwareTimer *tim, SoftTimerCallback cb);
void SoftwareTimer_arm(SoftwareTimer *tim, SoftTimerType type, uint32_t length);
void SoftwareTimer_start( SoftTimerList *list, SoftwareTimer *tim);
void SoftwareTimer_tick(SoftTimerList *tim_list);
void SoftwareTimer_wait_for(SoftwareTimer *tim);


#endif /* SOFT_TIMERS_SOFTWARETIMER_H_ */

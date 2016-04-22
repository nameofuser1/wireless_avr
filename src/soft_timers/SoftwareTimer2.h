/*
 * SoftwareTimer0.h
 *
 *  Created on: 01 дек. 2015 г.
 *      Author: kripton
 */

#ifndef SOFT_TIMERS_SOFTWARETIMER2_H_
#define SOFT_TIMERS_SOFTWARETIMER2_H_

#include "SoftwareTimer.h"
#include "stm32f10x.h"

#define TIMER2_PRESCALER 36000-1

SoftTimerList soft_timer2;

void SoftwareTimer2_init(void);
void SoftwareTimer2_start(void);
void SoftwareTimer2_stop(void);
void SoftwareTimer2_set_duration(uint16_t duration);



#endif /* SOFT_TIMERS_SOFTWARETIMER2_H_ */

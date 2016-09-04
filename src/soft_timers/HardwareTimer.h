/*
 * HardwareTimer.h
 *
 *  Created on: 3 сент. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_SOFT_TIMERS_HARDWARETIMER_H_
#define SRC_SOFT_TIMERS_HARDWARETIMER_H_

#include <stm32f10x.h>
#include "SoftwareTimer.h"

typedef  LinkedList HardwareTimerList;

typedef struct {
	HardwareTimerList *timers;
	TIM_TypeDef *io_tim;
} HardwareTimerResources;

/* Function definitions */
typedef void (*init_func_t)(uint16_t prescaler);
typedef void (*start_func_t)(void);
typedef void (*stop_func_t)(void);
typedef void (*set_duration_func_t)(uint16_t duration);
typedef void (*add_timer_func_t)(SoftwareTimer *tim);
typedef void (*remove_timer_func_t)(SoftwareTimer *tim);
typedef void (*delay_func_t)(uint32_t ms);

typedef struct {
	init_func_t Init;
	start_func_t Start;
	stop_func_t Stop;
	set_duration_func_t SetDuration;
	add_timer_func_t AddTimer;
	remove_timer_func_t RemoveTimer;
	delay_func_t Delay;
} HardwareTimerDriver;


void HardwareTimer_Init(HardwareTimerResources *timer, uint16_t prescaler);
void HardwareTimer_Start(HardwareTimerResources *timer);
void HardwareTimer_Stop(HardwareTimerResources *timer);
void HardwareTimer_SetDuration(HardwareTimerResources *timer, uint16_t duration);
void HardwareTimer_AddTimer(HardwareTimerResources *h_timer, SoftwareTimer *s_timer);
void HardwareTimer_RemoveTimer(HardwareTimerResources *h_timer, SoftwareTimer *s_timer);
void HardwareTimer_DelayMs(HardwareTimerResources *timer, uint32_t ms);


#endif /* SRC_SOFT_TIMERS_HARDWARETIMER_H_ */

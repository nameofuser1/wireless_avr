/*
 * SoftwareTimer0.c
 *
 *  Created on: 01 дек. 2015 г.
 *      Author: kripton
 */

#include "SoftwareTimer2.h"

void SoftwareTimer2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->PSC = TIMER2_PRESCALER;				//clock prescaler
	TIM2->DIER |= TIM_DIER_UIE;					//update interrupt enable
	TIM2->CR1 &= ~TIM_CR1_OPM;					//periodic mode
}


void SoftwareTimer2_start(void)
{
	TIM2->CR1 |= TIM_CR1_CEN;
}


void SoftwareTimer2_stop(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
}


void SoftwareTimer2_set_duration(uint16_t duration)
{
	TIM2->ARR = duration;
}


void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		SoftwareTimer_tick(&soft_timer2);
		TIM2->SR &= ~TIM_SR_UIF;
	}

}



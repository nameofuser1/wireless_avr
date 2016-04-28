/*
 * stm32_it.c
 *
 *  Created on: Apr 26, 2016
 *      Author: kript0n
 */
#include <stm32f10x.h>
#include <stm32f10x_exti.h>

#include "controller.h"

static uint8_t controller_initialized = 0;

void EXTI4_IRQHandler(void)
{
	if(controller_initialized)
	{
		CONTROLLER_DeInit();
	}
	else
	{
		CONTROLLER_init();
	}

	/* Clear interrupt flag */
	EXTI->PR |= EXTI_PR_PR4;
}

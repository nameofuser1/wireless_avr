/*
 * mcu.c
 *
 *  Created on: Feb 2, 2017
 *      Author: kript0n
 */

#include "mcu.h"
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>


void MCU_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef reset;
	GPIO_StructInit(&reset);

	reset.GPIO_Mode = GPIO_Mode_Out_PP;
	reset.GPIO_Pin = MCU_RESET_PIN;
	reset.GPIO_Speed = GPIO_Speed_10MHz;

	GPIO_Init(MCU_RESET_PORT, &reset);
}


void MCU_DeInit(void)
{
	GPIO_InitTypeDef reset;
	reset.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	reset.GPIO_Pin = MCU_RESET_PIN;

	GPIO_Init(MCU_RESET_PORT, &reset);
}

/*
 * UART.c
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: kripton
 */

#include <periph/usart.h>
#include "stm32f10x_usart.h"


void USART_SendArray(USART_TypeDef *USART, const uint8_t *arr, const uint32_t size)
{
	for(uint32_t i=0; i<size; i++)
	{
		while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
		USART_SendData(USART, arr[i]);
	}
}

void USART_SendString(USART_TypeDef *USART, const char *pucBuffer)
{
    uint8_t c;
    while((c = *pucBuffer++))
    {
        while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
        USART_SendData(USART, c);
    }
}

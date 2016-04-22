/*
 * UART.h
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: kripton
 */

#ifndef PERIPH_USART_H_
#define PERIPH_USART_H_

#include "stm32f10x.h"
#include <stdbool.h>

void 	USART_SendArray(USART_TypeDef *USART, const uint8_t *arr, const uint32_t size);
void 	USART_SendString(USART_TypeDef *USART, const char *pucBuffer);

#endif /* PERIPH_USART_H_ */

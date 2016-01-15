/*
 * UART.h
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: kripton
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f10x.h"
#include <stdbool.h>

#define USART3_BUF_SIZE 256

void 	USART1_init(void);
void 	USART3_init(void);
uint8_t USART3_read(void);
bool 	USART3_is_empty(void);
void 	USART_SendArray(USART_TypeDef *USART, const uint8_t *arr, const uint32_t size);
void 	USART_SendString(USART_TypeDef *USART, const char *pucBuffer);

#endif /* UART_H_ */

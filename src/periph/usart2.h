/*
 * usart2.h
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */

#ifndef PERIPH_USART2_H_
#define PERIPH_USART2_H_

#include "stm32f10x.h"
#include <stm32f10x_usart.h>
#include <stdbool.h>

void 		USART2_init(void);
uint8_t 	USART2_read(void);
bool 		USART2_tx_array(uint8_t *data, uint8_t len);
bool 		USART2_is_empty(void);
uint32_t 	USART2_available(void);
bool 		USART2_overflow(void);


#endif /* PERIPH_USART2_H_ */

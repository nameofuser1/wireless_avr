/*
 * usart1.h
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#ifndef PERIPH_USART1_H_
#define PERIPH_USART1_H_

#include "stm32f10x.h"
#include <stm32f10x_usart.h>
#include <stdbool.h>

void 		USART1_init(void);
uint8_t 	USART1_read(void);
bool 		USART1_tx_array(uint8_t *data, uint8_t len);
bool 		USART1_is_empty(void);
uint32_t 	USART1_available(void);
bool 		USART1_overflow(void);


#endif /* PERIPH_USART1_H_ */

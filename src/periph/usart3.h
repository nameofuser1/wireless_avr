/*
 * usart3.h
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#ifndef PERIPH_USART3_H_
#define PERIPH_USART3_H_

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "periph/usart.h"
#include <stdbool.h>

void 		USART1_init(void);
void 		USART3_init(void);
uint8_t 	USART3_read(void);
bool 		USART3_tx_array(uint8_t *data, uint8_t len);
bool 		USART3_is_empty(void);
uint32_t 	USART3_available(void);
bool 		USART3_overflow(void);


#endif /* PERIPH_USART3_H_ */

/*
 * system.h
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_SYSTEM_SYSTEM_H_
#define SRC_SYSTEM_SYSTEM_H_

#include <inttypes.h>

#define SystemTimer			TIMER2_Driver
#define SYSTEM_TIMER_IRQn	TIM2_IRQn

void 	system_init(void);
void	system_error(char *msg);
void	memory_error(char *msg);
void 	io_error(char *msg);

void* 	sys_malloc(int32_t size);

void	delay(uint32_t ms);

#endif /* SRC_SYSTEM_SYSTEM_H_ */

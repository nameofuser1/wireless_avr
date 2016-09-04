/*
 * system.h
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_SYSTEM_H_
#define SRC_SYSTEM_H_

#include <inttypes.h>

#define SystemTimer	TIMER2_Driver

void 	system_init(void);
void	system_error(char *msg);
void	memory_error(char *msg);
void 	io_error(char *msg);

void* 	sys_malloc(int32_t size);

#endif /* SRC_SYSTEM_H_ */

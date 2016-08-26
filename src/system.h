/*
 * system.h
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_SYSTEM_H_
#define SRC_SYSTEM_H_

#define SYSTEM_ERROR		0
#define SYSTEM_ERROR_MEM	1
#define SYSTEM_ERROR_IO		2
#define SYSTEM_ERRORS_NUM	3

#include <inttypes.h>

void 	critical_error(uint8_t err, char *msg);
void	system_error(char *msg);
void	memory_error(char *msg);
void 	io_error(char *msg);

void* 	sys_malloc(int32_t size);

#endif /* SRC_SYSTEM_H_ */

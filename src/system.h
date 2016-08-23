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
#define SYSTEM_ERRORS_NUM	2

void 	critical_error(char *msg);
void* 	sys_malloc(size_t size);

#endif /* SRC_SYSTEM_H_ */

/*
 * CRC.h
 *
 *  Created on: 07 апр. 2016 г.
 *      Author: kripton
 */

#ifndef CRC32_H_
#define CRC32_H_

#include <stm32f10x.h>

void 	 crc32_init(void);
uint32_t crc32_native(uint8_t *bfr, uint32_t len);

#endif /* CRC32_H_ */

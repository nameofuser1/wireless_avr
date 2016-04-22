/*
 * CRC.c
 *
 *  Created on: 07 апр. 2016 г.
 *      Author: kripton
 */

#include <crc32.h>
#include <stm32f10x_crc.h>
#include <stdio.h>
#include <inttypes.h>

#define CRC32_POLY   0x04C11DB7

static uint32_t crc32_table[256];

void crc32_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN;

	uint32_t i, j;
	uint32_t c;
	for (i = 0; i < 256; ++i)
	{
		c = i << 24;
		for (j = 8; j > 0; --j)
		{
				c = (c & 0x80000000) ? (c << 1) ^ CRC32_POLY : (c << 1);
		}

		crc32_table[i] = c;
	}
}


uint32_t crc32_native(uint8_t *bfr, uint32_t len) {

	CRC_ResetDR();

    uint32_t l = len / 4;
    uint32_t *p = (uint32_t*)bfr;
    uint32_t x = p[l];
    uint32_t crc = 0;

    while(l--)
    {
            crc = CRC_CalcCRC(*p++);
    }

    switch(len & 3)
    {
            case 1: crc = CRC_CalcCRC(x & 0x000000FF); break;
            case 2: crc = CRC_CalcCRC(x & 0x0000FFFF); break;
            case 3: crc = CRC_CalcCRC(x & 0x00FFFFFF); break;
    }

    return crc;
}

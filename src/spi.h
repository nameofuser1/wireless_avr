/*
 * spi.h
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: kripton
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f10x.h"
#include <stdbool.h>

void 	SPI_init(SPI_TypeDef *SPI);
void 	SPI_enable(SPI_TypeDef *SPI);
void 	SPI_disable(SPI_TypeDef *SPI);
void 	SPI_write(SPI_TypeDef *SPI, uint8_t data);
uint8_t SPI_read(SPI_TypeDef *SPI);
bool 	SPI_busy(SPI_TypeDef *SPI);
bool	SPI_TX_is_empty(SPI_TypeDef *SPI);
bool	SPI_RX_not_empty(SPI_TypeDef *SPI);


#endif /* SPI_H_ */

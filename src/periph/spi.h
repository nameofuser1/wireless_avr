/*
 * spi.h
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: kripton
 */

#ifndef PERIPH_SPI_H_
#define PERIPH_SPI_H_

#include "stm32f10x.h"
#include <stdbool.h>

void 	SPI1_init(void);
void 	SPI1_enable(void);
void 	SPI1_disable(void);
void 	SPI1_write(uint8_t data);
bool 	SPI1_busy(void);
bool	SPI1_TX_is_empty(void);
bool	SPI1_RX_not_empty(void);

uint8_t		SPI1_available(void);
bool 		SPI1_tx_array(uint16_t *data, uint8_t len);
uint16_t	SPI1_read(void);


#endif /* PERIPH_SPI_H_ */

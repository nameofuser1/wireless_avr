/*
 * spi.c
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: kripton
 */

#include <periph/spi.h>


void SPI_init(SPI_TypeDef *SPI)
{

/*
	SPI1->CR1 |=  SPI_CR1_BR_1;		//prescaler 8
	SPI1->CR1 |=  SPI_CR1_SSM;		//Software slave management
	SPI1->CR1 |=  SPI_CR1_SSI;		//NSS is HIGH
	SPI1->CR1 |=  SPI_CR2_SSOE;		//NSS is slave select
	SPI1->CR1 |=  SPI_CR1_MSTR;		//Master
	SPI1->CR1 &=  ~SPI_CR1_CPOL;	//SCK is low in idle
	SPI1->CR1 &=  ~SPI_CR1_CPHA;	//Capturing on falling edge
	SPI1->CR1 &=  ~SPI_CR1_DFF;		//8 bit frame format
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;	//MSB first
*/

		SPI_InitTypeDef spi;
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
		spi.SPI_Mode = SPI_Mode_Master;
		spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		spi.SPI_FirstBit = SPI_FirstBit_MSB;
		spi.SPI_DataSize = SPI_DataSize_8b;
		spi.SPI_NSS = SPI_NSS_Soft;
		spi.SPI_CPOL = SPI_CPOL_Low;
		spi.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
		SPI_Init(SPI1, &spi);
		SPI_Cmd(SPI1, ENABLE);

	//SPI->CR2 |= SPI_CR2_TXEIE;		//TX buffer empty interrupt
	//SPI->CR2 |= SPI_CR2_RXNEIE;		//RX buffer not empty interrupt

	//SPI->CR2 |= SPI_CR2_TXDMAEN;	//TX buffer DMA enable
	//SPI->CR2 |= SPI_CR2_RXDMAEN;	//RX buffer DMA enable
}


void SPI_enable(SPI_TypeDef *SPI)
{
	SPI->CR1 |= SPI_CR1_SPE;
}


void SPI_disable(SPI_TypeDef *SPI)
{
	while(!(SPI->SR & SPI_SR_RXNE));
	while(!(SPI->SR & SPI_SR_TXE));
	while((SPI->SR & SPI_SR_BSY));
	SPI->CR1 &= ~SPI_CR1_SPE;
}


void SPI_write(SPI_TypeDef *SPI, uint8_t data)
{
	while(SPI->SR & SPI_SR_BSY);
	SPI->DR = data;
}


uint8_t SPI_read(SPI_TypeDef *SPI)
{
	return (uint8_t)(SPI->DR & 0xFF);
}


bool SPI_busy(SPI_TypeDef *SPI)
{
	return	(SPI->SR & SPI_SR_BSY);
}


bool SPI_TX_is_empty(SPI_TypeDef *SPI)
{
	return (SPI->SR & SPI_SR_TXE);
}


bool SPI_RX_not_empty(SPI_TypeDef *SPI)
{
	return (SPI->SR & SPI_SR_RXNE);
}

/*
 * spi.c
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: kripton
 */

#include <periph/spi.h>


#define SPI1_RX_BUFFER_SIZE 256
#define SPI1_TX_BUFFER_SIZE 256

static uint16_t spi_rx_buffer[SPI1_RX_BUFFER_SIZE];
static uint16_t spi_tx_buffer[SPI1_TX_BUFFER_SIZE];

static uint8_t rx_counter = 0;
static uint8_t rx_wr_pointer = 0;
static uint8_t rx_rd_pointer = 0;

static uint8_t tx_counter = 0;
static uint8_t tx_wr_pointer = 0;
static uint8_t tx_rd_pointer = 0;


void SPI1_init(void)
{
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
}


void SPI1_enable(void)
{
	SPI1->CR1 |= SPI_CR1_SPE;
}


void SPI1_disable(void)
{
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
	//while(!(SPI->SR & SPI_SR_RXNE));
	//while(!(SPI->SR & SPI_SR_TXE));
	while((SPI1->SR & SPI_SR_BSY));
	SPI1->CR1 &= ~SPI_CR1_SPE;
}


void SPI1_write(uint8_t data)
{
	while(SPI1->SR & SPI_SR_BSY);
	SPI1->DR = data;
}


bool SPI1_busy(void)
{
	return	(SPI1->SR & SPI_SR_BSY);
}


bool SPI1_TX_is_empty(void)
{
	return (SPI1->SR & SPI_SR_TXE);
}


bool SPI_RX_not_empty(void)
{
	return (SPI1->SR & SPI_SR_RXNE);
}


uint8_t SPI1_available(void)
{
	return rx_counter;
}

/*
 * *************************************
 * !!!Check for available data first!!!
 * *************************************
 */
uint16_t SPI1_read(void)
{
	uint16_t data = 0;
	if(rx_counter > 0)
	{
		data = spi_rx_buffer[rx_rd_pointer];
		rx_rd_pointer = (rx_rd_pointer == SPI1_RX_BUFFER_SIZE - 1) ? 0 : rx_rd_pointer+1;
		rx_counter -= 1;
	}

	return data;
}


bool SPI1_tx_array(uint16_t *data, uint8_t len)
{
	if(tx_counter + len < SPI1_TX_BUFFER_SIZE)
	{
		for(uint32_t i = 0; i<len; i++)
		{
			spi_tx_buffer[tx_wr_pointer] = data[i];
			tx_wr_pointer = (tx_wr_pointer == SPI1_TX_BUFFER_SIZE - 1) ? 0 : tx_wr_pointer+1;
		}

		tx_counter += len;
		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);

		return true;
	}

	return false;
}



void SPI1_IRQHandler(void)
{
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE))
	{
		spi_rx_buffer[rx_wr_pointer] =  SPI_I2S_ReceiveData(SPI1);
		rx_wr_pointer = (rx_wr_pointer == SPI1_RX_BUFFER_SIZE - 1) ? 0 : rx_wr_pointer+1;
		rx_counter++;

		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
	}

	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE))
	{
		if(tx_counter > 0)
		{
			SPI_I2S_SendData(SPI1, spi_tx_buffer[tx_rd_pointer]);
			tx_rd_pointer = (tx_rd_pointer == SPI1_TX_BUFFER_SIZE - 1) ? 0 : tx_rd_pointer;
			tx_counter--;

			SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);
		}
		else
		{
			SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
		}
	}

	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_OVR))
	{
		//overrun error
		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_OVR);
	}


}

/*
 * usart2.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include "usart2.h"

/*
 * usart3.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include <periph/usart3.h>
#include <stdio.h>

#define USART2_RX_BUF_SIZE 128
#define USART2_TX_BUF_SIZE 128

/*
 * RX buffer and pointers
 */
static uint8_t  usart2_rx_buffer[USART2_RX_BUF_SIZE];
static uint32_t usart2_rx_wr_pointer = 0;
static uint32_t usart2_rx_rd_pointer = 0;
static uint32_t usart2_rx_counter = 0;

/*
 * TX buffer and pointers
 */
static uint8_t	usart2_tx_buffer[USART2_TX_BUF_SIZE];
static uint32_t usart2_tx_wr_pointer = 0;
static uint32_t usart2_tx_rd_pointer = 0;
static uint32_t usart2_tx_counter = 0;

static bool overflow = false;


void USART2_init(void)
{
    /* PA2 TX push-pull output, 10MHz */
    GPIOB->CRH &= ~GPIO_CRH_CNF10;
    GPIOB->CRH |= GPIO_CRH_CNF10_1;  //Alternative function, push-pull as CNF9_0 = 0
    GPIOB->CRH |= GPIO_CRH_MODE10_0; //10 MHz

    /* PA3 RX HI-Z input, 10MHz*/
    GPIOB->CRH &= ~GPIO_CRH_CNF11;
    GPIOB->CRH |= GPIO_CRH_CNF11_0; //HI_Z
    GPIOB->CRH &= ~GPIO_CRH_MODE11; //Clear MODE <=> INPUT

    //CircularBuffer_alloc(&usart3_buffer, 1024);

    /* Enable RCC USART3 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
	usart.USART_BaudRate = 115200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &usart);
}


uint8_t USART2_read(void)
{
	if(usart2_rx_counter > 0)
	{
		uint8_t byte = usart2_rx_buffer[usart2_rx_rd_pointer];
		usart2_rx_rd_pointer = (usart2_rx_rd_pointer == USART2_RX_BUF_SIZE-1) ? 0 : usart2_rx_rd_pointer+1;
		usart2_rx_counter--;
		return byte;
	}
	return 0;
}


bool USART2_tx_array(uint8_t *data, uint8_t len)
{
	if(len + usart2_tx_counter > USART2_TX_BUF_SIZE)
	{
		return false;
	}

	for(uint32_t i=0; i<len; i++)
	{
		usart2_tx_buffer[usart2_tx_wr_pointer] = data[i];
		usart2_tx_wr_pointer = (usart2_tx_wr_pointer == USART2_TX_BUF_SIZE-1) ? 0 : usart2_tx_wr_pointer+1;
	}
	usart2_tx_counter += len;

	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	return true;
}


bool USART2_is_empty(void)
{
	return (usart2_rx_counter == 0);
}

uint32_t USART2_available(void)
{
	return usart2_rx_counter;
}


bool USART2_overflow(void)
{
	bool ovf = overflow;
	overflow = false;
	return ovf;
}


void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		if((USART2->SR & (USART_FLAG_NE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)) == 0)
		{
			uint8_t data = (uint8_t)(USART_ReceiveData(USART2) & 0xFF);
			usart2_rx_buffer[usart2_rx_wr_pointer] = data;
			usart2_rx_wr_pointer = (usart2_rx_wr_pointer == USART2_RX_BUF_SIZE-1) ? 0 : usart2_rx_wr_pointer+1;

			if(++usart2_rx_counter == USART2_RX_BUF_SIZE)
			{
				overflow = true;
			}
		}
		else
		{
			/* Temp error */
			USART_ReceiveData(USART2);
		}
	}

	if(USART_GetITStatus(USART2, USART_IT_TC) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
	{
		if(usart2_tx_counter > 0)
		{
			USART_SendData(USART2, usart2_tx_buffer[usart2_tx_rd_pointer]);
			usart2_tx_counter--;
			usart2_tx_rd_pointer = (usart2_tx_rd_pointer == USART2_TX_BUF_SIZE-1) ? 0 : usart2_tx_rd_pointer+1;
		}
		else
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}
}



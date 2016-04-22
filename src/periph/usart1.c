/*
 * usart1.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include <periph/usart1.h>


#define USART1_RX_BUF_SIZE 128
#define USART1_TX_BUF_SIZE 128

/*
 * RX buffer and pointers
 */
static uint8_t  usart1_rx_buffer[USART1_RX_BUF_SIZE];
static uint32_t usart1_rx_wr_pointer = 0;
static uint32_t usart1_rx_rd_pointer = 0;
static uint32_t usart1_rx_counter = 0;

/*
 * TX buffer and pointers
 */
static uint8_t	usart1_tx_buffer[USART1_TX_BUF_SIZE];
static uint32_t usart1_tx_wr_pointer = 0;
static uint32_t usart1_tx_rd_pointer = 0;
static uint32_t usart1_tx_counter = 0;

static bool overflow = false;



void USART1_init(void)
{
    /* PA9 TX push-pull output, 10MHz */
    GPIOA->CRH &= ~GPIO_CRH_CNF9;
    GPIOA->CRH |= GPIO_CRH_CNF9_1;  //Alternative function, push-pull as CNF9_0 = 0
    GPIOA->CRH |= GPIO_CRH_MODE9_0; //10 MHz

    /* PA10 RX HI-Z input */
    GPIOA->CRH &= ~GPIO_CRH_CNF10;
    GPIOA->CRH |= GPIO_CRH_CNF10_0; //HI_Z
    GPIOA->CRH &= ~GPIO_CRH_MODE10; //Clear MODE <=> INPUT

    /* Remap USART1 from PA9/PA10 to PB6/PB7 */
    //AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    /* PB6 TX alternative push-pull output 10MHz */
    //GPIOB->CRL &= ~GPIO_CRL_CNF6;
    //GPIOB->CRL |= GPIO_CRL_CNF6_1;
    //GPIOB->CRL |= GPIO_CRL_MODE6_0;

    /* PB7 RX HI-Z input */
    //GPIOB->CRL &= ~GPIO_CRL_CNF7;
    //GPIOB->CRL |= GPIO_CRL_CNF7_0;
    //GPIOB->CRL &= ~GPIO_CRL_MODE7;

    /* Enable RCC USART1 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
	usart.USART_BaudRate = 115200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &usart);
}



uint8_t USART1_read(void)
{
	if(usart1_rx_counter > 0)
	{
		uint8_t byte = usart1_rx_buffer[usart1_rx_rd_pointer];
		usart1_rx_rd_pointer = (usart1_rx_rd_pointer == USART1_RX_BUF_SIZE-1) ? 0 : usart1_rx_rd_pointer+1;
		usart1_rx_counter--;
		return byte;
	}
	return 0;
}


bool USART1_tx_array(uint8_t *data, uint8_t len)
{
	if(len + usart1_tx_counter > USART1_TX_BUF_SIZE)
	{
		return false;
	}

	for(uint32_t i=0; i<len; i++)
	{
		usart1_tx_buffer[usart1_tx_wr_pointer] = data[i];
		usart1_tx_wr_pointer = (usart1_tx_wr_pointer == USART1_TX_BUF_SIZE-1) ? 0 :usart1_tx_wr_pointer+1;
	}
	usart1_tx_counter += len;

	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	return true;
}


bool USART1_is_empty(void)
{
	return (usart1_rx_counter == 0);
}

uint32_t USART1_available(void)
{
	return usart1_rx_counter;
}


bool USART1_overflow(void)
{
	bool ovf = overflow;
	overflow = false;
	return ovf;
}


void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		if((USART1->SR & (USART_FLAG_NE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)) == 0)
		{
			uint8_t data = (uint8_t)(USART_ReceiveData(USART1) & 0xFF);
			usart1_rx_buffer[usart1_rx_wr_pointer] = data;
			usart1_rx_wr_pointer = (usart1_rx_wr_pointer == USART1_RX_BUF_SIZE-1) ? 0 : usart1_rx_wr_pointer+1;

			if(++usart1_rx_counter == USART1_RX_BUF_SIZE)
			{
				overflow = true;
			}
		}
		else
		{
			/* Temp error */
			USART_ReceiveData(USART1);
		}
	}

	if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
	}

	if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
	{
		if(usart1_tx_counter > 0)
		{
			USART_SendData(USART1, usart1_tx_buffer[usart1_tx_rd_pointer]);
			usart1_tx_counter--;
			usart1_tx_rd_pointer = (usart1_tx_rd_pointer == USART1_TX_BUF_SIZE-1) ? 0 : usart1_tx_rd_pointer+1;
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
}


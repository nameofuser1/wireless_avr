/*
 * usart3.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include <periph/usart3.h>
#include <stdio.h>
#include "soft_timers/SoftwareTimer.h"
#include "soft_timers/SoftwareTimer2.h"


#define USART3_RX_BUF_SIZE 512
#define USART3_TX_BUF_SIZE 512

/*
 * RX buffer and pointers
 */
static uint8_t  usart3_rx_buffer[USART3_RX_BUF_SIZE];
static uint32_t usart3_rx_wr_pointer = 0;
static uint32_t usart3_rx_rd_pointer = 0;
static uint32_t usart3_rx_counter = 0;

/*
 * TX buffer and pointers
 */
static uint8_t	usart3_tx_buffer[USART3_TX_BUF_SIZE];
static uint32_t usart3_tx_wr_pointer = 0;
static uint32_t usart3_tx_rd_pointer = 0;
static uint32_t usart3_tx_counter = 0;

/*
 * Transmission status corresponding variable
 */
//static void				counter_task(void);
//static SoftwareTimer 	counter_timer;
static uint32_t 		time_counter;
static uint32_t			last_char_timestamp;


void USART3_init(void)
{
    /* TX push-pull output, 10MHz */
    GPIOB->CRH &= ~GPIO_CRH_CNF10;
    GPIOB->CRH |= GPIO_CRH_CNF10_1;  //Alternative function, push-pull as CNF9_0 = 0
    GPIOB->CRH |= GPIO_CRH_MODE10_0; //10 MHz

    /* RX HI-Z input, 10MHz*/
    GPIOB->CRH &= ~GPIO_CRH_CNF11;
    GPIOB->CRH |= GPIO_CRH_CNF11_0; //HI_Z
    GPIOB->CRH &= ~GPIO_CRH_MODE11; //Clear MODE <=> INPUT

    /* Enable RCC USART3 */
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    USART_InitTypeDef usart;

    USART_StructInit(&usart);
	usart.USART_BaudRate = 57600;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &usart);

	//SoftwareTimer_init(&counter_timer);
	//SoftwareTimer_arm(&counter_timer, Timer_Repeat, 1);
	//SoftwareTimer_add_cb(&counter_timer, counter_task);
	//SoftwareTimer_start(&soft_timer2, &counter_timer);
}

/*
 * *******************************************
 * Read one byte from buffer
 * *******************************************
 */
uint8_t USART3_read(void)
{
	if(usart3_rx_counter > 0)
	{
		uint8_t byte = usart3_rx_buffer[usart3_rx_rd_pointer];
		usart3_rx_rd_pointer = (usart3_rx_rd_pointer == USART3_RX_BUF_SIZE-1) ? 0 : usart3_rx_rd_pointer+1;
		usart3_rx_counter--;
		return byte;
	}
	return 0;
}


/*
 * *********************************************
 * Transmit array with interrupts
 * *********************************************
 */
bool USART3_tx_array(uint8_t *data, uint8_t len)
{
	if(len + usart3_tx_counter > USART3_TX_BUF_SIZE)
	{
		return false;
	}

	for(uint32_t i=0; i<len; i++)
	{
		usart3_tx_buffer[usart3_tx_wr_pointer] = data[i];
		usart3_tx_wr_pointer = (usart3_tx_wr_pointer == USART3_TX_BUF_SIZE-1) ? 0 :usart3_tx_wr_pointer+1;
	}
	usart3_tx_counter += len;

	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	return true;
}


bool USART3_is_empty(void)
{
	return (usart3_rx_counter == 0);
}


uint32_t USART3_available(void)
{
	return usart3_rx_counter;
}


/*
 * ****************************************************
 * If last char was received later than 5ms ago.
 * We suppose that transmission is completed
 * ****************************************************
 */
bool USART3_transmission_status(void)
{
	return ((time_counter - last_char_timestamp) < 5);
}

/*
 * **********************************
 * Called as counter_timer callback
 * **********************************
 */
/*
static void counter_task(void)
{
	time_counter++;
}
*/

/*
 * **********************************************************
 * last_char_timestamp contains time in millisecond when
 * last char was read. Time since counter_timer was started.
 *
 * Everything else is just writing to or reading from
 * corresponding circular buffers.
 * **********************************************************
 */
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		if((USART3->SR & (USART_FLAG_NE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)) == 0)
		{
			last_char_timestamp = time_counter;

			uint8_t data = (uint8_t)(USART_ReceiveData(USART3) & 0xFF);
			usart3_rx_buffer[usart3_rx_wr_pointer] = data;
			usart3_rx_wr_pointer = (usart3_rx_wr_pointer == USART3_RX_BUF_SIZE-1) ? 0 : usart3_rx_wr_pointer+1;
		}
		else
		{
			/* Temp error */
			USART_ReceiveData(USART3);
		}

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if(USART_GetITStatus(USART3, USART_IT_TC) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_TC);
	}

	if(USART_GetITStatus(USART3, USART_IT_TXE) == SET)
	{
		if(usart3_tx_counter > 0)
		{
			USART_SendData(USART3, usart3_tx_buffer[usart3_tx_rd_pointer]);
			usart3_tx_counter--;
			usart3_tx_rd_pointer = (usart3_tx_rd_pointer == USART3_TX_BUF_SIZE-1) ? 0 : usart3_tx_rd_pointer+1;
		}
		else
		{
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}
}


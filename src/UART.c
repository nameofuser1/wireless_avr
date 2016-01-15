/*
 * UART.c
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: kripton
 */

#include "UART.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

static uint8_t  usart3_buffer[USART3_BUF_SIZE];
static uint32_t usart3_wr_pointer = 0;
static uint32_t usart3_rd_pointer = 0;
static uint32_t remaining_data = 0;

void USART1_init(void)
{
    /* TX push-pull output, 10MHz */
    GPIOA->CRH &= ~GPIO_CRH_CNF9;
    GPIOA->CRH |= GPIO_CRH_CNF9_1;  //Alternative function, push-pull as CNF9_0 = 0
    GPIOA->CRH |= GPIO_CRH_MODE9_0; //10 MHz

    /* RX HI-Z input, 10MHz*/
    GPIOA->CRH &= ~GPIO_CRH_CNF10;
    GPIOA->CRH |= GPIO_CRH_CNF10_0; //HI_Z
    GPIOA->CRH &= ~GPIO_CRH_MODE10; //Clear MODE <=> INPUT

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

    //CircularBuffer_alloc(&usart3_buffer, 1024);

    /* Enable RCC USART3 */
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
	usart.USART_BaudRate = 19200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &usart);
}


uint8_t USART3_read(void)
{
	if(remaining_data > 0)
	{
		remaining_data--;
		uint8_t byte = usart3_buffer[usart3_rd_pointer];
		usart3_rd_pointer = (usart3_rd_pointer == USART3_BUF_SIZE-1) ? 0 : usart3_rd_pointer+1;
		return byte;
	}
	return 0;
}


bool USART3_is_empty(void)
{
	return (remaining_data == 0);
}


void USART3_IRQHandler(void)
{
	//printf("UART3 interrupt\r\n");
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		uint8_t data = (uint16_t)(USART_ReceiveData(USART3) & 0xFF);
		usart3_buffer[usart3_wr_pointer] = data;
		//printf("Got byte: 0x%02x\r\n", data);
		usart3_wr_pointer = (usart3_wr_pointer == USART3_BUF_SIZE-1) ? 0 : usart3_wr_pointer+1;
		remaining_data++;
	}
}


void USART_SendArray(USART_TypeDef *USART, const uint8_t *arr, const uint32_t size)
{
	for(uint32_t i=0; i<size; i++)
	{
		while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
		USART_SendData(USART, arr[i]);
	}
}

void USART_SendString(USART_TypeDef *USART, const char *pucBuffer)
{
    uint8_t c;
    while((c = *pucBuffer++))
    {
        while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
        USART_SendData(USART, c);
    }
}

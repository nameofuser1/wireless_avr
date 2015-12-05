/*
 * UART.c
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: kripton
 */

#include "UART.h"
#include "stm32f10x_usart.h"


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

    /* Enable RCC USART1 */
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


void USART_SendString(USART_TypeDef *USART, const char *pucBuffer)
{
    uint8_t c;
    while((c = *pucBuffer++))
    {
        while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
        USART_SendData(USART, c);
    }
}



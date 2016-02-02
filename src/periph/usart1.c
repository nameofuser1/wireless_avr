/*
 * usart1.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include <periph/usart1.h>


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


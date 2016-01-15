/*
 * avr_flasher.c
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */
#include <common/CircularBuffer.h>
#include "avr_flasher.h"
#include "UART.h"
#include <stdio.h>

static uint8_t PROG_STATE = STATE_READY;
static CircularBuffer send_buffer;
static CircularBuffer recv_buffer;


/*
 * **************************************
 * Initialize everything that needed for
 * programming
 * **************************************
 */
void AVRFlasher_init(void)
{
	CircularBuffer_alloc(&send_buffer, SEND_BUFFER_SIZE);
	CircularBuffer_alloc(&recv_buffer, RECV_BUFFER_SIZE);

	SPI_init(SPI1);
	//TIM1_init(RESET_TIM_PRESCALER);
}



/*
 * *****************************************
 * Applies init instruction sequence for
 * programming.
 * *****************************************
 */
void AVRFlasher_start(void)
{
	SPI_enable(SPI1);
	AVRFlasher_reset_enable();
	PROG_STATE = STATE_WAIT_AT_READY;
	//TIM1_set_duration(WAIT_AT_READY_DURATION);
	//TIM1_start();
	//AVRFlasher_reset(RESET_DURATION);
}


/*
 * *************************************
 * Return current programmer state
 * *************************************
 */
uint8_t AVRFlasher_get_state(void)
{
	return PROG_STATE;
}


/*
 * *******************************************
 * Send reset pulse with given duration
 * Pulse is stopped in the interrupt handler
 * *******************************************
 */
void AVRFlasher_reset_pulse(uint16_t duration)
{
	AVRFlasher_reset_enable();
	PROG_STATE = STATE_RESET_PULSE;
	//TIM1_set_duration(RESET_DURATION);
	//TIM1_start();
}



/*
 * *******************************************
 * Send command to AVR
 * *******************************************
 */
void AVRFlasher_send_command(AvrCommand *command, uint8_t *res)
{
	for(int i=0; i<4; i++)
	{
		uint8_t cmd_byte = *(((uint8_t*)command)+i);
		SPI_write(SPI1, cmd_byte);

		while(!(SPI1->SR & SPI_SR_RXNE));
		res[i] = SPI1->DR;
	}
}


void AVRFlasher_prog_enable(void)
{
	//AvrCommand command;
	//command.b1 = AT16_PROG_EN_B1;
	//command.b2 = AT16_PROG_EN_B2;
	//command.b3 = AT16_PROG_EN_B3;
	//command.b4 = AT16_PROG_EN_B4;

	//AVRFlasher_send_command(&command);

	PROG_STATE = STATE_WAIT_DATA;
	//TIM1_set_duration(WAIT_DATA_DURATION);
	//TIM1_start();
	while((!SPI_RX_not_empty(SPI1)) && (PROG_STATE == STATE_WAIT_DATA));
	//TIM1_stop();
	while(SPI_RX_not_empty(SPI1))
	{
		uint8_t data = SPI_read(SPI1);
		printf("Read data: 0x%02X\r\n", data);
	}

	AVRFlasher_reset_disable();
}


void TIM1_UP_IRQHandler(void)
{
	switch(PROG_STATE)
	{
		case STATE_RESET_PULSE:
			printf("STATE_RESET TIM\r\n");
			GPIOA->BSRR |= GPIO_BSRR_BR3;
			PROG_STATE = STATE_WAIT_AT_READY;
			//TIM1_set_duration(WAIT_AT_READY_DURATION);
			//TIM1_start();
			break;

		case STATE_WAIT_AT_READY:
			printf("STATE_WAIT_AT_READY\r\n");
			PROG_STATE = STATE_READY;
			break;

		case STATE_WAIT_DATA:
			printf("STATE MISSING DATA\r\n");
			PROG_STATE = STATE_MISSING_DATA;
			break;
	}
	TIM1->SR &= ~TIM_SR_UIF;
}


void SPI1_IRQHandler(void)
{
	if(SPI1->SR & SPI_SR_RXNE)
	{
		uint8_t data = SPI_read(SPI1);
		printf("Get data: 0x%02x\r\n", data);
		CircularBuffer_put(&recv_buffer, (void*)&data);
		SPI1->SR &= ~SPI_SR_RXNE;
	}
	else if(SPI1->SR & SPI_SR_TXE)
	{
		printf("TX buffer is empty\r\n");
		uint8_t byte = *(uint8_t*)CircularBuffer_get(&send_buffer);
		SPI1->DR = byte;
		printf("Send byte 0x%02x\r\n", byte);
		SPI1->SR &= ~SPI_SR_TXE;
	}
}





void AVRFlasher_reset_enable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BR3;
}

void AVRFlasher_reset_disable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BS3;
}

/*
 * avr_flasher.c
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */
#include "avr_flasher.h"
#include <stdio.h>
#include <periph/spi.h>

static uint8_t PROG_STATE = STATE_READY;

/*
 * **************************************
 * Initialize everything that needed for
 * programming
 * **************************************
 */
void AVRFlasher_init(void)
{
	SPI1_init();
}



/*
 * *****************************************
 * Applies init instruction sequence for
 * programming.
 * *****************************************
 */
void AVRFlasher_start(void)
{
	SPI1_enable();
	AVRFlasher_reset_enable();
	PROG_STATE = STATE_WAIT_AT_READY;
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
}



/*
 * *******************************************
 * Send command to AVR
 * *******************************************
 */
void AVRFlasher_send_command(AvrCommand *command, uint8_t *res)
{
	//printf("Send command: ");
	//for(int i=0; i<4; i++) printf("0x%02x ", *(((uint8_t*)command)+i));
	//printf("\r\n");
	for(int i=0; i<4; i++)
	{
		uint8_t cmd_byte = *(((uint8_t*)command)+i);
		SPI1_write(cmd_byte);

		while(!(SPI1->SR & SPI_SR_RXNE));
		res[i] = SPI1->DR;
	}
}


void AVRFlasher_reset_enable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BR8;
}

void AVRFlasher_reset_disable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BS8;
}

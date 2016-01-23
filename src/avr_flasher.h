/*
 * avr_flasher.h
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */

#ifndef AVR_FLASHER_H_
#define AVR_FLASHER_H_

#include "avr_commands.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "spi.h"

#define ACK_PACKET_BYTE 	0xAA
#define INIT_PACKET_BYTE	0xFF
#define STOP_PACKET_BYTE	0x01
#define ERROR_PACKET_BYTE	0xEE

/*
 * APB2 has 36MHz frequency so TIM will have 1ms tick
 * Reset duration is 25ms according to AT16 datasheet(at least 20 ms)
 */
#define RESET_TIM_PRESCALER		36000-1
#define RESET_DURATION			10
#define WAIT_DATA_DURATION		5
#define WAIT_AT_READY_DURATION  25

/*
 *
 */
#define SEND_BUFFER_SIZE		128
#define RECV_BUFFER_SIZE		128


/*
 * Avr flasher states for interrupts
 */
#define STATE_READY				0x00
#define STATE_RESET_PULSE		0x01
#define STATE_WAIT_DATA			0x02
#define STATE_WRITE_DATE		0x03
#define STATE_WAIT_AT_READY		0x04
#define STATE_MISSING_DATA		0x05
#define STATE_PROG_FINISHED		0x06



void 		AVRFlasher_init(void);
void 		AVRFlasher_start(void);
uint8_t 	AVRFlasher_get_state(void);
void 		AVRFlasher_reset(uint16_t duration);
void 	AVRFlasher_send_command(AvrCommand *command, uint8_t *res);
void 		AVRFlasher_prog_enable(void);


void AVRFlasher_reset_enable(void);
void AVRFlasher_reset_disable(void);

#endif /* AVR_FLASHER_H_ */

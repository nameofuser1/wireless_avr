/*
 * protocol.h
 *
 *  Created on: Aug 25, 2016
 *      Author: kript0n
 */

#ifndef SRC_PROTOCOL_H_
#define SRC_PROTOCOL_H_

#define MAX_PACKET_LENGTH	300

/* Send packet types */
#define ACK_PACKET_BYTE 				0xAA
#define USART_PACKET_BYTE				0xBB
#define ERROR_PACKET_BYTE				0xEE
#define MEMORY_PACKET_BYTE				0xCC

/* Receive packet types */
#define LOG_PACKET_BYTE					0x10
#define AVR_PROG_INIT_BYTE				0x11
#define STOP_PROGRAMMER_PACKET_BYTE		0x22
#define CMD_PACKET_BYTE					0x33
#define RESET_PACKET_BYTE				0x44
#define PROG_MEM_PACKET_BYTE			0x55
#define USART_INIT_PACKET_BYTE			0x66
#define LOAD_MCU_INFO_BYTE				0x77
#define READ_MEM_PACKET_BYTE			0x88
#define PGM_ENABLE_PACKET_BYTE			0x99
#define LOAD_NETWORK_INFO_BYTE			0x12
#define NETWORK_INFO_LOADED_BYTE		0x13
#define NONE_PACKET_BYTE 				0x00

#define INTERNAL_ERROR_BYTE		0x00
#define WRONG_PACKET_BYTE		0x01

/* Header structure */
#define SIZE_FIELD_OFFSET		0
#define SIZE_FIELD_SIZE			2
#define TYPE_FIELD_OFFSET		(SIZE_FIELD_OFFSET + SIZE_FIELD_SIZE)
#define TYPE_FIELD_SIZE			1
#define CRC_FIELD_SIZE     		4
#define PACKET_HEADER_SIZE		(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE)
#define PACKET_RESERVED_BYTES	(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE + CRC_FIELD_SIZE)

/* Reset packet structure */
#define RESET_BYTE_OFFSET	0
#define RESET_ENABLE 		1
#define RESET_DISABLE		0

/* USART Packet structure */
#define USART_BAUDRATE_BYTE_OFFSET		(0)
#define USART_PARITY_BYTE_OFFSET		(USART_BAUDRATE_BYTE_OFFSET+1)
#define USART_DATA_BITS_BYTE_OFFSET		(USART_PARITY_BYTE_OFFSET+1)
#define USART_STOP_BITS_BYTE_OFFSET		(USART_DATA_BITS_BYTE_OFFSET+1)

/* USART Config bytes */
#define USART_PARITY_EVEN		0xC0
#define USART_PARITY_ODD		0xC1
#define USART_PARITY_NONE		0xC2
#define USART_DATA_BITS_8		0xC3
#define USART_DATA_BITS_9		0xC4
#define USART_STOP_BITS_1		0xC5
#define USART_STOP_BITS_2		0xC6
#define USART_BAUDRATE_9600		0xC7
#define USART_BAUDRATE_19200	0xC8
#define USART_BAUDRATE_59600	0xC9
#define USART_BAUDRATE_115200	0xCA

/* USART errors */
#define USART_PARITY_ERROR_BYTE 	0xE0
#define USART_FRAME_ERROR_BYTE		0xE1
#define USART_DATA_BITS_ERROR_BYTE	0xE2
#define USART_STOP_BITS_ERROR_BYTE	0xE3
#define USART_BAUDRATE_ERROR_BYTE	0xE4


static uint32_t get_size_from_header(uint8_t *header)
{
	uint32_t size = 0;

	for(uint8_t i=0; i<SIZE_FIELD_SIZE; i++)
	{
		size |= (header[SIZE_FIELD_OFFSET + i] << 8*(SIZE_FIELD_SIZE - 1 - i));
	}

	return size;
}

#endif /* SRC_PROTOCOL_H_ */

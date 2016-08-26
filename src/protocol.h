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
#define ACK_PACKET_BYTE 		0xAA
#define USART_PACKET_BYTE		0xBB
#define ERROR_PACKET_BYTE		0xEE
#define MEMORY_PACKET_BYTE		0xCC

/* Receive packet types */
#define LOG_PACKET_BYTE					0x10
#define AVR_PROG_INIT_BYTE				0x11
#define STOP_PROGRAMMER_PACKET_BYTE		0x22
#define CMD_PACKET_BYTE					0x33
#define RESET_PACKET_BYTE				0x44
#define PROG_MEM_PACKET_BYTE			0x55
#define USART_INIT_PACKET_BYTE			0x66
#define LOAC_MCU_INFO_BYTE				0x77
#define READ_MEM_PACKET_BYTE			0x88
#define PGM_ENABLE_PACKET_BYTE			0x99

#define LOAD_NETWORK_INFO_BYTE			0x12
#define NETWORK_INFO_LOADED_BYTE		0x13

#define NONE_PACKET_BYTE 		0x00

#define INTERNAL_ERROR_BYTE		0x00
#define WRONG_PACKET_BYTE		0x01

#define SIZE_FIELD_SIZE			2
#define SIZE_FIELD_OFFSET		0
#define TYPE_FIELD_OFFSET		2
#define TYPE_FIELD_SIZE			1
#define CRC_FIELD_SIZE     		4
#define PACKET_HEADER_SIZE		(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE)
#define PACKET_RESERVED_BYTES	(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE + CRC_FIELD_SIZE)

/* Reset packet parameters */
#define RESET_ENABLE 	1
#define RESET_DISABLE	0


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

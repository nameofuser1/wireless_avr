/*
 * PacketManager.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include "crc32.h"
#include "PacketManager.h"
#include "esp8266.h"
#include "system.h"
#include "common/logging.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/* Send packet types */
#define ACK_PACKET_BYTE 		0xAA
#define USART_PACKET_BYTE		0xBB
#define ERROR_PACKET_BYTE		0xEE
#define MEMORY_PACKET_BYTE		0xCC

/* Receive packet types */
#define LOG_PACKET_BYTE			0x10
#define PROG_INIT_BYTE			0x11
#define STOP_PROGRAMMER_PACKET_BYTE		0x22
#define CMD_PACKET_BYTE			0x33
#define RESET_PACKET_BYTE		0x44
#define PROG_MEM_PACKET_BYTE	0x55
#define USART_INIT_PACKET_BYTE	0x66
#define AVR_PROG_INIT_BYTE		0x77
#define READ_MEM_PACKET_BYTE	0x88
#define PGM_ENABLE_PACKET_BYTE	0x99
#define NETWORK_INFO_LOADED		0x98

#define NONE_PACKET_BYTE 		0x00

#define INTERNAL_ERROR_BYTE		0x00
#define WRONG_PACKET_BYTE		0x01

#define SIZE_FIELD_SIZE			2
#define TYPE_FIELD_SIZE			1
#define CRC_FIELD_SIZE     		4
#define PACKET_HEADER_SIZE		(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE)
#define PACKET_RESERVED_BYTES	(SIZE_FIELD_SIZE + TYPE_FIELD_SIZE + CRC_FIELD_SIZE)

#define MAX_PACKET_LENGTH	300

#define ERROR_PACKET_LEN	3
#define ERROR_LEN			1

#define PACKETS_BUF_SIZE 	10
#define PARSING_BUF_SIZE	512

#define PACKETS_TYPES_NUMBER	(NONE_PACKET)

/* For easy logging */
static char* packet_names[PACKETS_TYPES_NUMBER] =
{
	"ProgInit", "Stop", 		"Cmd", 			"Reset", 		"Error", 	"ProgMem", "ReadMem",
	"Usart", 	"UsartInit", 	"AvrProgInit",  "PGM enable", 	"ACK", 		"Memory",  "LOG",
};

/* Static methods definitions */
static bool 		_check_crc(uint8_t *data, uint32_t len);
static uint32_t 	_packet_crc(Packet packet);
static uint8_t 		_get_packet_type_byte(PacketType type);
static PacketType 	_get_packet_type(uint8_t type_byte);



void PacketManager_init(void)
{
	crc32_init();
}


/*
 * *****************************************************
 * Parse packets from an array
 *
 * As we don't receive any error packets then returning
 * ERROR_PACKET means error while parsing. Check error
 * by packet->data[0] byte.
 * *****************************************************
 */
Packet PacketManager_parse(uint8_t *packet_buffer, uint32_t data_len)
{
	uint32_t packet_len = 0;
	for(uint8_t i=0; i<SIZE_FIELD_SIZE; i++)
	{
		packet_len |= ((packet_buffer[i]) << 8*(SIZE_FIELD_SIZE - i - 1));
	}

	uint8_t type_byte = packet_buffer[SIZE_FIELD_SIZE];	// next byte after size field
	PacketType packet_type = _get_packet_type(type_byte);

	if(packet_type == NONE_PACKET)
	{
		return PacketManager_error_packet(PACKET_TYPE_ERROR);
	}

	if(packet_type != LOG_PACKET)
	{
		if(!_check_crc(packet_buffer, data_len))
		{
			return PacketManager_error_packet(PACKET_CRC_ERROR);
		}
	}

	uint32_t data_length =  data_len - PACKET_RESERVED_BYTES;
	uint8_t data_offset = PACKET_HEADER_SIZE;

	uint8_t *packet_data = packet_buffer+data_offset;
	Packet packet = PacketManager_CreatePacket(packet_data, data_len, packet_type);

	return packet;
}


/*
 * **************************************
 * Release data array and packet pointer
 * **************************************
 */
void PacketManager_free(Packet packet)
{
	if(packet->type != ACK_PACKET)
	{
		free(packet->data);
	}

	free(packet);
}


Packet PacketManager_CreateErrorPacket(uint8_t error)
{
	uint8_t data[1] = {error};
	return PacketManager_CreatePacket(data, 1, ERROR_PACKET);
}


/*
 * *************************************
 * Create packet from array
 * *************************************
 */
Packet	PacketManager_CreatePacket(uint8_t *data, uint16_t data_len, PacketType type)
{
	if(data_len > MAX_PACKET_LENGTH)
	{
		critical_error(SYSTEM_ERROR, "Packet length is over max");
	}

	Packet packet = (Packet)sys_malloc(sizeof(struct _packet));
	packet->type = type;
	packet->data_length = data_len;

	if(data_len != 0)
	{
		packet->data = (uint8_t*)sys_malloc(sizeof(uint8_t) * (packet->data_length));
		memcpy((packet->data), data, data_len*sizeof(uint8_t));
	}
	else
	{
		packet->data = NULL;
	}

	packet->crc = _packet_crc(packet);
	return packet;
}


/*
 * Transform packet into array
 * Arguments:
 * 		Packet --- packet to transform
 * 		uint32_t --- bytes in the resulting array
 */
uint8_t* PacketManager_Packet2Buf(Packet packet, uint32_t *bytes)
{
	uint32_t buffer_length = packet->data_length+PACKET_RESERVED_BYTES;
	*bytes = buffer_length;

	uint8_t *buf = (uint8_t*)sys_malloc(sizeof(buffer_length));
	uint8_t type_byte = _get_packet_type_byte(packet->type);

	if(type_byte == NONE_PACKET_BYTE)
	{
		critical_error(SYSTEM_ERROR, "Wrong packet type");
	}

	buf[0] = (packet->data_length >> 8) & 0xFF;
	buf[1] = (packet->data_length) & 0xFF;
	buf[2] = (uint8_t)type_byte;

	memcpy(buf+PACKET_HEADER_SIZE, packet->data, packet->data_length);

	/* If using memcpy have to use htonl as arm has little-endian */
	for(uint32_t i=0; i<CRC_FIELD_SIZE; i++)
	{
		buf[buffer_length-CRC_FIELD_SIZE+i] = (packet->crc >> (24-i*8)) & 0xFF;
	}


	return buf;
}


static uint32_t _packet_crc(Packet packet)
{
	uint8_t crc_buf[packet->data_length + PACKET_HEADER_SIZE];
	crc_buf[0] = (packet->data_length >> 8) & 0xFF;
	crc_buf[1] = packet->data_length & 0xFF;

	int8_t packet_type_byte = _get_packet_type_byte(packet->type);
	if(packet_type_byte == -1)
	{
		critical_error(SYSTEM_ERROR, "Wrong packet byte");
	}

	crc_buf[2] = (uint8_t)packet_type_byte;
	memcpy(crc_buf+PACKET_HEADER_SIZE, packet->data, packet->data_length);

	return crc32_native(crc_buf, packet->data_length + PACKET_HEADER_SIZE);
}


static bool _check_crc(uint8_t *data, uint32_t len)
{
	uint32_t calc_crc = crc32_native(data, len-CRC_FIELD_SIZE);

	uint32_t true_crc = 0;
	for(uint32_t i=0; i<CRC_FIELD_SIZE; i++)
	{
		true_crc |= (data[len-(i+1)] << 8*i);
	}

	return calc_crc == true_crc;
}


static uint8_t _get_packet_type_byte(PacketType type)
{
	switch(type)
	{
		case PROG_INIT_PACKET:
			return PROG_INIT_BYTE;

		case STOP_PROGRAMMER_PACKET:
			return STOP_PROGRAMMER_PACKET_BYTE;

		case CMD_PACKET:
			return CMD_PACKET_BYTE;

		case RESET_PACKET:
			return RESET_PACKET_BYTE;

		case ERROR_PACKET:
			return ERROR_PACKET_BYTE;

		case PROG_MEM_PACKET:
			return PROG_MEM_PACKET_BYTE;

		case READ_MEM_PACKET:
			return READ_MEM_PACKET_BYTE;

		case USART_PACKET:
			return USART_PACKET_BYTE;

		case USART_INIT_PACKET:
			return USART_INIT_PACKET_BYTE;

		case AVR_PROG_INIT_PACKET:
			return AVR_PROG_INIT_BYTE;

		case PGM_ENABLE_PACKET:
			return PGM_ENABLE_PACKET_BYTE;

		case ACK_PACKET:
			return ACK_PACKET_BYTE;

		case MEMORY_PACKET:
			return MEMORY_PACKET_BYTE;

		case LOG_PACKET:
			return LOG_PACKET_BYTE;

		default:
			return NONE_PACKET_BYTE;
	}
}


static PacketType _get_packet_type(uint8_t type_byte)
{
	switch(type_byte)
	{
		case PROG_INIT_BYTE:
			return PROG_INIT_PACKET;

		case STOP_PROGRAMMER_PACKET_BYTE:
			return STOP_PROGRAMMER_PACKET;

		case CMD_PACKET_BYTE:
			return CMD_PACKET;

		case RESET_PACKET_BYTE:
			return RESET_PACKET;

		case ERROR_PACKET_BYTE:
			return ERROR_PACKET;

		case PROG_MEM_PACKET_BYTE:
			return PROG_MEM_PACKET;

		case READ_MEM_PACKET_BYTE:
			return READ_MEM_PACKET;

		case USART_PACKET_BYTE:
			return USART_PACKET;

		case USART_INIT_PACKET_BYTE:
			return USART_INIT_PACKET;

		case AVR_PROG_INIT_BYTE:
			return AVR_PROG_INIT_PACKET;

		case PGM_ENABLE_PACKET_BYTE:
			return PGM_ENABLE_PACKET;

		case ACK_PACKET_BYTE:
			return ACK_PACKET;

		case MEMORY_PACKET_BYTE:
			return MEMORY_PACKET;

		case LOG_PACKET_BYTE:
			return LOG_PACKET;

		default:
			return NONE_PACKET;
	}
}




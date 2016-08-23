/*
 * PacketManager.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include <crc32.h>
#include "PacketManager.h"
#include "esp8266.h"
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
#define STOP_PACKET_BYTE		0x22
#define CMD_PACKET_BYTE			0x33
#define RESET_PACKET_BYTE		0x44
#define PROG_MEM_PACKET_BYTE	0x55
#define USART_INIT_PACKET_BYTE	0x66
#define AVR_PROG_INIT_BYTE		0x77
#define READ_MEM_PACKET_BYTE	0x88
#define PGM_ENABLE_PACKET_BYTE	0x99
#define NETWORK_INFO_LOADED		0x98

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

static char* packet_names[PACKETS_TYPES_NUMBER] =
{
	"ProgInit", "Stop", 		"Cmd", 			"Reset", 		"Error", 	"ProgMem", "ReadMem",
	"Usart", 	"UsartInit", 	"AvrProgInit",  "PGM enable", 	"ACK", 		"Memory",  "LOG",
};


static bool PacketManager_check_crc(uint8_t *data, uint32_t len);

static Packet 	packets[PACKETS_BUF_SIZE];
static uint8_t 	packets_available = 0;
static uint8_t 	packets_wr_pointer = 0;
static uint8_t	packets_rd_pointer = 0;

static uint8_t 		parsing_buf[PARSING_BUF_SIZE];
static PacketType	parsing_packet_type = NONE_PACKET;
static uint16_t 	parsing_packet_length = 0;
static uint16_t 	parsing_len = 0;


void PacketManager_init(void)
{
	crc32_init();
}

/*
 * ****************************************************
 * Parse packets from an array and save them to buffer
 * Use PacketManager_get_packet to get read packets
 * Be sure that array length is packet size scalable
 * ****************************************************
 */
bool PacketManager_parse(void)
{
	uint32_t available = ESP8266_available();

	if(available > 0)
	{
		if(parsing_packet_length == 0)
		{
			if(available >= 2)
			{
				parsing_buf[parsing_len++] = ESP8266_read();
				parsing_buf[parsing_len++] = ESP8266_read();

				parsing_packet_length |= (parsing_buf[0] << 8) & 0xFF00;
				parsing_packet_length |= (parsing_buf[1] & 0xFF);

				available -= 2;
				//printf("Got packet length: %" PRIu16 "\r\n", parsing_packet_length);
			}
			else
			{
				return true;
			}
		}

		if(parsing_packet_type == NONE_PACKET)
		{
			if(available > 0)
			{
				uint8_t type_byte = ESP8266_read();
				available--;
				parsing_buf[parsing_len++] = type_byte;

				switch(type_byte)
				{
					case PROG_INIT_BYTE:
						parsing_packet_type = PROG_INIT_PACKET;
						break;

					case AVR_PROG_INIT_BYTE:
						parsing_packet_type = AVR_PROG_INIT_PACKET;
						break;

					case STOP_PACKET_BYTE:
						parsing_packet_type = STOP_PROGRAMMER_PACKET;
						break;

					case CMD_PACKET_BYTE:
						parsing_packet_type = CMD_PACKET;
						break;

					case RESET_PACKET_BYTE:
						parsing_packet_type = RESET_PACKET;
						break;

					case PROG_MEM_PACKET_BYTE:
						parsing_packet_type = PROG_MEM_PACKET;
						break;

					case READ_MEM_PACKET_BYTE:
						parsing_packet_type = READ_MEM_PACKET;
						break;

					case USART_INIT_PACKET_BYTE:
						parsing_packet_type = USART_INIT_PACKET;
						break;

					case PGM_ENABLE_PACKET_BYTE:
						parsing_packet_type = PGM_ENABLE_PACKET;
						break;

					case ERROR_PACKET_BYTE:
						parsing_packet_type = ERROR_PACKET;
						break;

					case LOG_PACKET_BYTE:
						parsing_packet_type = LOG_PACKET;
						break;

					default:
						printf("Got wrong packet byte: 0x%02x %c\r\n", type_byte, type_byte);
						return false;
				}
			}
			else
			{
				return true;
			}
		}


		if((available > 0) && (parsing_len < parsing_packet_length))
		{
			ESP8266_read_arr(&(parsing_buf[parsing_len]), available);
			parsing_len += available;
		}

		//printf("Got %" PRIu16 " bytes\r\n", parsing_len);
		if(parsing_len == parsing_packet_length)
		{
			if(packets_available+1 > PACKETS_BUF_SIZE)
			{
				return false;
			}

			if(parsing_packet_type != LOG_PACKET)
			{
				if(!PacketManager_check_crc(parsing_buf, parsing_packet_length))
				{
					printf("CRC doesn't match. Packet %s\r\n", packet_names[parsing_packet_type]);
					ESP8266_SendError(WRONG_PACKET_BYTE);
				}


				if(parsing_packet_type == ERROR_PACKET && parsing_buf[3] == WRONG_PACKET_BYTE)
				{
					ESP8266_SendLastPacket();
				}
				else if(parsing_packet_type != LOG_PACKET)
				{
					uint32_t data_length =  parsing_packet_length - PACKET_RESERVED_BYTES;
					uint8_t data_offset = PACKET_HEADER_SIZE;

					uint8_t *packet_data = malloc(sizeof(uint8_t)*(data_length));
					memcpy(packet_data, parsing_buf+data_offset, data_length);

					Packet packet = {
							.type = parsing_packet_type,
							.data_length = parsing_packet_length - PACKET_RESERVED_BYTES,
							.data = packet_data
					};

					packets[packets_wr_pointer] = packet;
					packets_wr_pointer = (packets_wr_pointer == PACKETS_BUF_SIZE-1) ? 0 : packets_wr_pointer+1;
					++packets_available;

					printf("Got %s packet\r\n", packet_names[packet->type]);
				}

			}
			else
			{
				int data_len = parsing_packet_length - PACKET_HEADER_SIZE;
				uint8_t data_offset = PACKET_HEADER_SIZE;
				printf("ESP LOG: %.*s", data_len, (char*)(parsing_buf+data_offset));
			}

			parsing_packet_length = 0;
			parsing_len = 0;
			parsing_packet_type = NONE_PACKET;
		}
		else if(!ESP8266_TransmissionStatus())
		{
			printf("Missing packet bytes\r\n");

			ESP8266_flush_rx();
			ESP8266_SendError(WRONG_PACKET_BYTE);
		}
	}

	return true;
}


bool PacketManager_available(void)
{
	return (packets_available > 0);
}

/*
 * ******************************************
 * Check for available packets first!!!
 * ******************************************
 */
Packet PacketManager_get_packet(void)
{
	Packet packet;
	if(packets_available > 0)
	{
		packet = packets[packets_rd_pointer];
		packets_rd_pointer = (packets_rd_pointer == PACKETS_BUF_SIZE-1) ? 0 : packets_rd_pointer+1;
		packets_available--;
	}

	return packet;
}


/*
 * *************************************
 * Check for available packets first!!!
 * *************************************
 */
PacketType	PacketManager_next_packet_type(void)
{
	if(packets_available > 0)
	{
		return packets[packets_rd_pointer]->type;
	}

	return 0;
}


/*
 * ***********************************
 * Release data array
 * ***********************************
 */
void PacketManager_free(Packet packet)
{
	if(packet->type != ACK_PACKET)
	{
		free(packet->data);
	}

	free(packet);
}


/*
 * ***************************************
 * Reset all pointers and counter
 * ***************************************
 */
void PacketManager_clear(void)
{
	packets_available = 0;
	packets_wr_pointer = 0;
	packets_rd_pointer = 0;
}


/*
 * *************************************
 * Creates packet to send
 * *************************************
 */
Packet	PacketManager_create_packet(uint8_t *data, uint16_t data_len, PacketType type)
{
	Packet packet = (Packet)malloc(sizeof(struct _packet));
	packet->data_length = data_len + PACKET_RESERVED_BYTES;

	if(packet->data_length > MAX_PACKET_LENGTH)
	{
		packet->type = NONE_PACKET;
		printf("Wrong packet length\r\n");
		return packet;
	}

	packet->data = (uint8_t*)malloc(sizeof(uint8_t) * (packet->data_length));
	packet->data[0] = (packet->data_length >> 8) & 0xFF;
	packet->data[1] = (packet->data_length & 0xFF);

	switch(type)
	{
		case CMD_PACKET:
			packet->type = CMD_PACKET;
			packet->data[2] = CMD_PACKET_BYTE;
			break;

		case ERROR_PACKET:
			packet->type = ERROR_PACKET;
			packet->data[2] = ERROR_PACKET_BYTE;
			break;

		case USART_PACKET:
			packet->type = USART_PACKET;
			packet->data[2] = USART_PACKET_BYTE;
			break;

		case ACK_PACKET:
			packet->type = ACK_PACKET;
			packet->data[2] = ACK_PACKET_BYTE;
			break;

		case MEMORY_PACKET:
			packet->type = MEMORY_PACKET;
			packet->data[2] = MEMORY_PACKET_BYTE;
			break;

		default:
			packet->type = NONE_PACKET;
			break;
	}

	if(data_len != 0)
	{
		uint32_t data_offset = PACKET_HEADER_SIZE;
		memcpy((packet->data + data_offset), data, data_len*sizeof(uint8_t));
	}

	uint32_t crc_offset = packet->data_length - CRC_FIELD_SIZE;
	uint32_t crc = crc32_native(packet->data, data_len+PACKET_HEADER_SIZE);

	/* If using memcpy have to use htonl as arm has little-endian */
	for(uint32_t i=0; i<CRC_FIELD_SIZE; i++)
	{
		packet->data[crc_offset+i] = (crc >> (24-i*8)) & 0xFF;
	}

	return packet;
}


/*
 * *************************************************
 *
 * *************************************************
 */
static bool PacketManager_check_crc(uint8_t *data, uint32_t len)
{
	uint32_t calc_crc = crc32_native(data, len-CRC_FIELD_SIZE);

	uint32_t true_crc = 0;
	for(uint32_t i=0; i<CRC_FIELD_SIZE; i++)
	{
		true_crc |= (data[len-(i+1)] << 8*i);
	}

	return calc_crc == true_crc;
}

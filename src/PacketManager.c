/*
 * PacketManager.c
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#include "PacketManager.h"
#include "avr_flasher.h"

#define PACKETS_BUF_SIZE 	10
#define PACKET_BYTES_NUM	6
#define PACKET_TYPE_DEFAULT	0x3F	//first 6 bits
#define PACKET_LENGTH		4

#define NULL (void*)0


static Packet 	packets[PACKETS_BUF_SIZE];
static uint8_t 	packets_available = 0;
static uint8_t 	packets_wr_pointer = 0;
static uint8_t	packets_rd_pointer = 0;

static PacketType get_type(uint8_t _type);


/*
 * ****************************************************
 * Parse packets from an array and save them to buffer
 * Use PacketManager_get_packet to get read packets
 * Be sure that array length is packet size scalable
 * ****************************************************
 */
bool PacketManager_parse(uint8_t *data, uint8_t len)
{
	if(len%PACKET_LENGTH != 0)
	{
		return false;
	}

	uint8_t type = PACKET_TYPE_DEFAULT;
	uint32_t packets_num = len/PACKET_LENGTH;
	for(uint32_t i=0; i<packets_num; i++)
	{
		uint32_t index = i*PACKET_LENGTH;
		Packet packet;
		for(uint32_t j=0; j<PACKET_LENGTH; j++)
		{
			packet.data[index+j] = data[index+j];

			if(data[i] != INIT_PACKET_BYTE)
			{
				type &= ~(1 << INIT_PACKET);
			}

			if(data[i] != STOP_PACKET_BYTE)
			{
				type &= ~(1 << STOP_PACKET);
			}

			if(data[i] != RESTART_PACKET_BYTE)
			{
				type &= ~(1 << RESTART_PACKET);
			}

			if(data[i] != RESET_PACKET_BYTE)
			{
				type &= ~(1 << RESET_PACKET);
			}

			if(data[i] != ERROR_PACKET_BYTE)
			{
				type &= ~(1 << ERROR_PACKET);
			}
		}
		packet.type = get_type(type);

		packets[packets_wr_pointer] = packet;
		packets_wr_pointer = (packets_wr_pointer == PACKETS_BUF_SIZE-1) ? 0 : packets_wr_pointer+1;

		if(++packets_available == PACKETS_BUF_SIZE)
		{
			return false;
		}
	}

	return true;
}


bool PacketManager_available(void)
{
	if(packets_available > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
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
		return packets[packets_rd_pointer].type;
	}

	return 0;
}

/*
 * *********************************************
 * One of the bits is used for one type
 * CMD bit is not reset so it tells truth only
 * when other bits are set to 0.
 * *********************************************
 */
static PacketType get_type(uint8_t _type)
{
	PacketType type;

	if((_type & ~(1<<CMD_PACKET)) == 0)
	{
		type = CMD_PACKET;
	}
	else if(_type & (1<<INIT_PACKET))
	{
		type = INIT_PACKET;
	}
	else if(_type & (1<<STOP_PACKET))
	{
		type = STOP_PACKET;
	}
	else if(_type & (1<<RESTART_PACKET))
	{
		type = RESTART_PACKET;
	}
	else if(_type & (1<<RESET_PACKET))
	{
		type = RESET_PACKET;
	}
	else if(_type & (1<<ERROR_PACKET))
	{
		type = ERROR_PACKET;
	}

	return type;
}

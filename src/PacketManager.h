/*
 * PacketManager.h
 *
 *  Created on: 27 янв. 2016 г.
 *      Author: kripton
 */

#ifndef PACKETMANAGER_H_
#define PACKETMANAGER_H_

#include "stm32f10x.h"
#include <stdbool.h>
#include "ProgrammerTypes.h"


typedef enum {AVR_PROG_INIT_PACKET=0, STOP_PROGRAMMER_PACKET, CMD_PACKET, RESET_PACKET, ERROR_PACKET,
		PROG_MEM_PACKET, READ_MEM_PACKET, USART_PACKET, USART_INIT_PACKET, LOAD_MCU_INFO_PACKET,
		PGM_ENABLE_PACKET, ACK_PACKET, MEMORY_PACKET, LOG_PACKET,
		LOAD_NET_INFO_PACKET, NONE_PACKET} PacketType;


typedef struct _packet {

	PacketType 	type;
	uint16_t 	data_length;
	uint8_t 	*data;
	uint32_t	crc;

} *Packet;


#define PACKET_CRC_ERROR 				0
#define PACKET_MEMORY_ERROR 			1
#define PACKET_LENGTH_ERROR				2
#define PACKET_TYPES_ERROR				3
#define PACKET_HEADER_IDLE_LINE_ERROR	4
#define PACKET_BODY_IDLE_LINE_ERROR		5
#define PACKET_RECEIVE_BUSY_ERROR		6
#define PACKET_RECEIVE_PARAMETER_ERROR	7
#define PACKET_SEND_BUSY_ERROR			8
#define PACKET_SEND_PARAMETER_ERROR		9
#define PACKET_UNKNOWN_DRIVER_ERROR		10


void		PacketManager_init(void);
Packet 		PacketManager_parse(uint8_t *buffer);
void		PacketManager_free(Packet packet);
Packet 		PacketManager_CreateErrorPacket(uint8_t err);
Packet		PacketManager_CreatePacket(uint8_t* data, uint16_t data_len, PacketType type);
uint8_t* 	PacketManager_Packet2Buf(Packet packet, uint32_t *bytes);
Packet		PacketManager_Copy(Packet packet);




#endif /* PACKETMANAGER_H_ */

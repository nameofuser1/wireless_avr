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


typedef enum {PROG_INIT_PACKET=0, STOP_PROGRAMMER_PACKET, CMD_PACKET, RESET_PACKET, ERROR_PACKET,
		PROG_MEM_PACKET, READ_MEM_PACKET, USART_PACKET, USART_INIT_PACKET, AVR_PROG_INIT_PACKET,
		PGM_ENABLE_PACKET, ACK_PACKET, MEMORY_PACKET, LOG_PACKET, NONE_PACKET} PacketType;


typedef struct _packet {

	PacketType 	type;
	uint16_t 	data_length;
	uint8_t 	*data;
	uint32_t	crc;

} *Packet;


#define PACKET_CRC_ERROR 		0
#define PACKET_MEMORY_ERROR 	1
#define PACKET_LENGTH_ERROR		2
#define PACKET_TYPE_ERROR		3


void		PacketManager_init(void);
Packet 		PacketManager_parse(uint8_t *buffer, uint32_t len);
void		PacketManager_free(Packet packet);
Packet 		PacketManager_CreateErrorPacket(uint8_t err);
Packet		PacketManager_CreatePacket(uint8_t* data, uint16_t data_len, PacketType type);
uint8_t* 	PacketManager_Packet2Buf(Packet packet, uint32_t *bytes);




#endif /* PACKETMANAGER_H_ */

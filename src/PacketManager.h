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

/* Send packet types */
#define ACK_PACKET_BYTE 		0xAA
#define USART_PACKET_BYTE		0xBB
#define ERROR_PACKET_BYTE		0xEE
#define MEMORY_PACKET_BYTE		0xCC

/* Receive packet types */
#define PROG_INIT_BYTE			0x11
#define STOP_PACKET_BYTE		0x22
#define CMD_PACKET_BYTE			0x33
#define RESET_PACKET_BYTE		0x44
#define PROG_MEM_PACKET_BYTE	0x55
#define USART_INIT_PACKET_BYTE	0x66
#define AVR_PROG_INIT_BYTE		0x77
#define READ_MEM_PACKET_BYTE	0x88
#define PGM_ENABLE_PACKET_BYTE	0x99


typedef enum {PROG_INIT_PACKET=0, STOP_PACKET, CMD_PACKET, RESET_PACKET, ERROR_PACKET,
		PROG_MEM_PACKET, READ_MEM_PACKET, USART_PACKET, USART_INIT_PACKET,
		AVR_PROG_INIT_PACKET, PGM_ENABLE_PACKET, ACK_PACKET, MEMORY_PACKET, NONE_PACKET} PacketType;

typedef struct {

	PacketType 	type;
	uint16_t 	data_length;
	uint8_t 	*data;

} Packet;


bool 		PacketManager_parse(void);
bool 		PacketManager_available(void);
Packet		PacketManager_get_packet(void);
PacketType	PacketManager_next_packet_type(void);
void		PacketManager_free(Packet packet);
void		PacketManager_clear(void);
Packet		PacketManager_create_packet(uint8_t* data, uint8_t data_len, PacketType type);





#endif /* PACKETMANAGER_H_ */

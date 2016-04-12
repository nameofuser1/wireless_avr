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


typedef enum {PROG_INIT_PACKET=0, STOP_PACKET, CMD_PACKET, RESET_PACKET, ERROR_PACKET,
		PROG_MEM_PACKET, READ_MEM_PACKET, USART_PACKET, USART_INIT_PACKET,
		AVR_PROG_INIT_PACKET, PGM_ENABLE_PACKET, ACK_PACKET, MEMORY_PACKET, LOG_PACKET, NONE_PACKET} PacketType;


typedef struct {

	PacketType 	type;
	uint16_t 	data_length;
	uint8_t 	*data;

} Packet;


void		PacketManager_init(void);
bool 		PacketManager_parse(void);
bool 		PacketManager_available(void);
Packet		PacketManager_get_packet(void);
PacketType	PacketManager_next_packet_type(void);
void		PacketManager_free(Packet packet);
void		PacketManager_clear(void);
Packet		PacketManager_create_packet(uint8_t* data, uint16_t data_len, PacketType type);





#endif /* PACKETMANAGER_H_ */

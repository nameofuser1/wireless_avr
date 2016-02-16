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


typedef enum {INIT_PACKET=0, STOP_PACKET, CMD_PACKET, RESTART_PACKET, RESET_PACKET, ERROR_PACKET} PacketType;

typedef struct {

	uint8_t 	data[4];
	PacketType 	type;

} Packet;


bool 		PacketManager_parse(uint8_t *buf, uint8_t len);
bool 		PacketManager_available(void);
Packet		PacketManager_get_packet(void);
PacketType	PacketManager_next_packet_type(void);



#endif /* PACKETMANAGER_H_ */

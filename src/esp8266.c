/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include "esp8266.h"
#include <stddef.h>
#include "periph/usart3.h"
#include "PacketManager.h"
#include <stdio.h>
#include "periph/usart.h"


bool ESP8266_SendPacket(Packet packet)
{
	if(packet.type != NONE_PACKET)
	{
		USART3_tx_array(packet.data, packet.data_length);
		return true;
	}

	return false;
}


bool ESP8266_SendAck(void)
{
	Packet ack_packet = PacketManager_create_packet(NULL, 0, ACK_PACKET);

	printf("Sending ack\r\n");
	for(uint32_t i=0; i<ack_packet.data_length; i++)
	{
		printf("0x%02x ", ack_packet.data[i]);
	}
	printf("\r\n");

	bool res = ESP8266_SendPacket(ack_packet);

	PacketManager_free(ack_packet);

	return res;
}


bool ESP8266_SendError(uint8_t error)
{
	uint8_t err[1] = {error};
	Packet err_packet = PacketManager_create_packet(err, 1, ERROR_PACKET);
	bool res = ESP8266_SendPacket(err_packet);

	PacketManager_free(err_packet);

	return res;
}


uint32_t ESP8266_available(void)
{
	return USART3_available();
}


uint8_t ESP8266_read(void)
{
	return USART3_read();
}



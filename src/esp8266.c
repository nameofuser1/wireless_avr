/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include "esp8266.h"
#include "periph/usart.h"
#include "periph/usart3.h"
#include "PacketManager.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


static Packet last_packet;

#define ESP_STATUS_GPIO 		GPIOB
#define ESP_STATUS_GPIO_IDR		GPIO_IDR_IDR4

#define ESP_STATUS_GPIO_READ()	(ESP_STATUS_GPIO->IDR & ESP_STATUS_GPIO_IDR)


void ESP8266_init(void)
{
	USART3_init();
    USART_Cmd(USART3, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}


void ESP8266_WaitForReady(void)
{
	for(volatile uint32_t i=0; i<2000000; i++);
	while(!ESP_STATUS_GPIO_READ())
	{
		for(volatile uint32_t i=0; i<1000000; i++);
	}
}


bool ESP8266_SendPacket(Packet packet)
{
	if(packet.type != NONE_PACKET)
	{

		if(packet.data != last_packet.data)
		{
			if(last_packet.data != NULL)
			{
				free(last_packet.data);
			}

			last_packet.data = (uint8_t*)malloc(sizeof(uint8_t)*packet.data_length);
			memcpy(last_packet.data, packet.data, packet.data_length);
			last_packet.data_length = packet.data_length;
		}
/*
		printf("Sending packet: ");
		for(uint16_t i=0; i<packet.data_length; i++)
		{
			printf("0x%02x ", packet.data[i]);
		}
		printf("\r\n");
*/
		USART3_tx_array(packet.data, packet.data_length);

		return true;
	}

	return false;
}


bool ESP8266_SendAck(void)
{
	Packet ack_packet = PacketManager_create_packet(NULL, 0, ACK_PACKET);
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


bool ESP8266_SendLastPacket(void)
{
	return ESP8266_SendPacket(last_packet);
}


bool ESP8266_TransmissionStatus(void)
{
	return USART3_transmission_status();
}


uint32_t ESP8266_available(void)
{
	return USART3_available();
}


uint8_t ESP8266_read(void)
{
	return USART3_read();
}


void ESP8266_flush_rx(void)
{
	USART3_flush_rx();
}



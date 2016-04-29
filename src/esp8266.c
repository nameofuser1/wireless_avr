/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include "esp8266.h"
#include "periph/usart1.h"
#include "periph/usart3.h"
#include "PacketManager.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static Packet last_packet;

#define ESP_STATUS_GPIO 		GPIOB
#define ESP_STATUS_GPIO_IDR		GPIO_IDR_IDR4
#define ESP_STATUS_GPIO_READ()	(ESP_STATUS_GPIO->IDR & ESP_STATUS_GPIO_IDR)

#define ESP_INFO_LOAD_GPIO				GPIOB
#define ESP_INFO_LOAD_GPIO_BS			GPIO_BSRR_BS5
#define ESP_INFO_LOAD_GPIO_BR			GPIO_BSRR_BR5
#define ESP_INFO_LOAD_GPIO_ENABLE()		(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BS)
#define ESP_INFO_LOAD_GPIO_DISABLE()	(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BR)


void ESP8266_init(void)
{
	USART3_init();
}


void ESP8266_DeInit(void)
{
	USART3_deinit();
}


void ESP8266_WaitForReady(void)
{
	for(volatile uint32_t i=0; i<2000000; i++);
	while(!ESP_STATUS_GPIO_READ())
	{
		for(volatile uint32_t i=0; i<1000000; i++);
	}
}


bool ESP8266_Ready(void)
{
	return ESP_STATUS_GPIO_READ();
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


void ESP8266_LoadNetworkData(void)
{
	/*
	 * 1 byte for packet size
	 * 1 byte for ssid len
	 * 1 byte for pwd len
	 * 1 null character byte
	 */
	if(USART1_available() > 4)
	{
		uint8_t available = USART1_available();

		printf("Got network info packet\r\n");

		Packet packet;
		packet.type = CMD_PACKET;
		packet.data = malloc(sizeof(uint8_t)*available);
		packet.data_length = available;

		uint8_t k = 0;
		while(available-- > 0)
		{
			packet.data[k++] = USART1_read();
		}

		//ESP_INFO_LOAD_GPIO_ENABLE();
		ESP8266_SendPacket(packet);

		free(packet.data);
	}
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


void ESP8266_flush_tx(void)
{
	USART3_flush_tx();
}



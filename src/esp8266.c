/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp8266.h"
#include "PacketManager.h"
#include <Driver_USART.h>

#define ESP_STATUS_GPIO 		GPIOB
#define ESP_STATUS_GPIO_IDR		GPIO_IDR_IDR4
#define ESP_STATUS_GPIO_READ()	(ESP_STATUS_GPIO->IDR & ESP_STATUS_GPIO_IDR)

#define ESP_INFO_LOAD_GPIO				GPIOB
#define ESP_INFO_LOAD_GPIO_BS			GPIO_BSRR_BS5
#define ESP_INFO_LOAD_GPIO_BR			GPIO_BSRR_BR5
#define ESP_INFO_LOAD_GPIO_ENABLE()		(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BS)
#define ESP_INFO_LOAD_GPIO_DISABLE()	(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BR)

#define ESP_Driver_Usart Driver_USART3
extern ARM_DRIVER_USART ESP_Driver_Usart;

#define ESP_USART_BAUDRATE 115200


static void usart_event_handler(ARM_USART_SignalEvent_t event)
{
	if(event & ARM_USART_EVENT_SEND_COMPLETE)
	{

	}


}

void ESP8266_init(void)
{
	ARM_USART_SignalEvent_t events =	(ARM_USART_EVENT_SEND_COMPLETE |
										 ARM_USART_EVENT_RECEIVE_COMPLETE |
										 ARM_USART_EVENT_RX_TIMEOUT);

	ESP_Driver_Usart->Initialize(usart_event_handler);
	ESP_Driver_Usart->Control(ARM_USART_MODE_ASYNCHRONOUS, ESP_USART_BAUDRATE);
	ESP_Driver_Usart->Control(ARM_USART_CONTROL_TX | ARM_USART_CONTROL_RX, 1);
}


void ESP8266_DeInit(void)
{
	ESP_Driver_Usart->Uninitialize();
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
		ESP_Driver_Usart->Send((void*)packet.data, packet.data_length);
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
	return true; //ESP8266_SendPacket(last_packet);
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
	return ESP_Driver_Usart->GetStatus()->tx_busy;
}


uint32_t ESP8266_available(void)
{
	return ESP_Driver_Usart->GetRxCount();
}


bool ESP8266_read_arr(uint8_t *buf, uint32_t len)
{
	return ESP_Driver_Usart->Receive((void*)buf, len);
}


void ESP8266_flush_rx(void)
{
	__NOP();
}


void ESP8266_flush_tx(void)
{
	__NOP();
}



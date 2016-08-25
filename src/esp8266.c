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

#include "stm32f10x.h"
#include "esp8266.h"
#include "system.h"
#include "PacketManager.h"
#include "common/CircularBuffer.h"
#include <Driver_USART.h>

#define ESP_STATUS_GPIO 		GPIOB
#define ESP_STATUS_GPIO_IDR		GPIO_IDR_IDR4
#define ESP_STATUS_GPIO_READ()	(ESP_STATUS_GPIO->IDR & ESP_STATUS_GPIO_IDR)

#define ESP_INFO_LOAD_GPIO				GPIOB
#define ESP_INFO_LOAD_GPIO_BS			GPIO_BSRR_BS5
#define ESP_INFO_LOAD_GPIO_BR			GPIO_BSRR_BR5
#define ESP_INFO_LOAD_GPIO_ENABLE()		(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BS)
#define ESP_INFO_LOAD_GPIO_DISABLE()	(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BR)

/* Import usart driver */
#define ESP_USART_Typedef	USART3
#define ESP_Driver_Usart 	Driver_USART3

extern ARM_DRIVER_USART ESP_Driver_Usart;

/* USART CONFIG */
#define ESP_USART_BAUDRATE 115200

/* Buffer sizes for saving packets */
#define OUTCOME_PACKETS_BUFFER_SIZE 10
#define INCOME_PACKETS_BUFFER_SIZE  5

/* Save outcome packets here */
static CircularBuffer outcome_packets_buffer;
static Packet processing_packet;

/* Save income packets here */
static CircularBuffer income_packets_buffer;

/* Buffers for reading */
#define HEADER_BUFFER_SIZE 	2
#define BODY_BUFFER_SIZE 	512

static uint8_t header_buffer[HEADER_BUFFER_SIZE];
static uint8_t body_buffer[BODY_BUFFER_SIZE];

/* State defines */
typedef enum 	{ESP_STATE_RECV_HEADERS, ESP_STATE_RECV_BODY} EspState;
static EspState esp_state = ESP_STATE_RECV_HEADERS;

/* Static methods */
static void receive_header(void);
static void receive_body(uint32_t body_len);


static void usart_event_handler(ARM_USART_SignalEvent_t event)
{
	if(event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		PacketManager_free(processing_packet);

		if(!CircularBuffer_is_empty(&outcome_packets_buffer))
		{
			processing_packet = CircularBuffer_get(&outcome_packets_buffer);

			if(ESP_Driver_Usart->Send((void*)(processing_packet->data), processing_packet->data_length) != ARM_DRIVER_OK)
			{
				critical_error("Can't send packet");
			}
		}
	}

	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		if(esp_state == ESP_STATE_RECV_HEADERS)
		{
			memcpy(body_buffer, header_buffer, HEADER_BUFFER_SIZE);
			uint16_t body_len = ((header_buffer[0] << 8) & 0xFF00) | (header_buffer[1] & 0xFF);

			receive_body(body_len);
			esp_state = ESP_STATE_RECV_BODY;
		}
		else
		{
			Packet packet = PacketManager_parse(body_buffer);

			if(packet->type == ERROR_PACKET)
			{
				if(packet->data[0] == PACKET_TYPE_ERROR)
				{
					critical_error("Unknown packet");
				}
				else if(packet->data[0] == PACKET_CRC_ERROR)
				{
					critical_error("Wrong crc");
				}
			}

			CircularBuffer_put(&income_packets_buffer, (void*)packet);

			receive_header();
			esp_state = ESP_STATE_RECV_HEADERS;
		}
	}

	if(event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		if(esp_state == ESP_STATE_RECV_HEADERS)
		{
			LOGGING_log("IDLE Line while receiving headers", LOG_DEBUG);
		}
		else
		{
			critical_error("IDLE Line while receiving packet body");
		}
	}
}


void ESP8266_Init(void)
{
	ESP_Driver_Usart->Initialize(usart_event_handler);
	ESP_Driver_Usart->Control(ARM_USART_MODE_ASYNCHRONOUS, ESP_USART_BAUDRATE);
	ESP_Driver_Usart->Control(ARM_USART_CONTROL_TX | ARM_USART_CONTROL_RX, 1);

	if(!CircularBuffer_alloc(&outcome_packets_buffer, OUTCOME_PACKETS_BUFFER_SIZE))
	{
		critical_error("Memory error. Can't allocate outcome buffer");
	}

	if(!CircularBuffer_alloc(&income_packets_buffer, INCOME_PACKETS_BUFFER_SIZE))
	{
		critical_error("Memory error. Can't allocate income buffer");
	}

	/* Starting receiving packets */
	receive_header();
}


void ESP8266_DeInit(void)
{
	ESP_Driver_Usart->Uninitialize();

	CirclularBuffer_free(&outcome_packets_buffer, true);
	CirclularBuffer_free(&income_packets_buffer, true);
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
	if(packet->type != NONE_PACKET)
	{
		if(CircularBuffer_is_empty(&outcome_packets_buffer))
		{
			processing_packet = packet;

			if(ESP_Driver_Usart->Send((void*)(packet->data), packet->data_length) != ARM_DRIVER_OK)
			{
				critical_error("Can't send packet.");
			}
		}
		else
		{
			CircularBuffer_put(&outcome_packets_buffer, packet);
		}

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


bool ESP8266_TransmissionStatus(void)
{
	return (ESP_Driver_Usart->GetStatus()->tx_busy == 1);
}


bool ESP8266_Available(void)
{
	return (!CircularBuffer_is_empty(&income_packets_buffer));
}


Packet ESP8266_GetPacket(void)
{
	if(ESP8266_Available())
	{
		return (Packet)CircularBuffer_get(&income_packets_buffer);
	}

	return PacketManager_create_packet(NULL, 0, NONE_PACKET);
}


/*
 * Receiving header.
 * Disable IDLE line interrupt as we can wait forever.
 */
static void receive_header(void)
{
	if(ESP_Driver_Usart->Receive(header_buffer, HEADER_BUFFER_SIZE) != ARM_DRIVER_OK)
	{
		critical_error("Can't receive packet");
	}

	ESP_USART_Typedef->CR1 &= ~(USART_CR1_IDLEIE);
}


/*
 * Receive packet body
 */
static void receive_body(uint32_t body_len)
{
	if(ESP_Driver_Usart->Receive(body_buffer+HEADER_BUFFER_SIZE, body_len) != ARM_DRIVER_OK)
	{
		critical_error("Can't receive buffer");
	}
}


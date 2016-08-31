/*
 * EspUpdater.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */


#include "EspUpdater.h"
#include "protocol.h"
#include "system.h"
#include "esp8266.h"
#include "common/logging.h"

#include <string.h>

#include <USART_STM32F10x.h>

/* Maximum buffer size */
#define INPUT_BUFFER_SIZE	50

static uint8_t in_buffer[INPUT_BUFFER_SIZE];

/* Reader state variables */
#define STATE_RECEIVE_HEADER 	0
#define STATE_RECEIVE_BODY		1

static uint8_t state = STATE_RECEIVE_HEADER;

/* Usart Driver_USART1 is set from out */
//static ARM_Driver_USART1_USART Driver_USART1;
#define EspUpdater_USART_IRQn	USART1_IRQn
extern ARM_DRIVER_USART 		Driver_USART1;


static void usart_callback(uint32_t event)
{
	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		if(state == STATE_RECEIVE_HEADER)
		{
			uint16_t size = get_size_from_header(in_buffer);

			state = STATE_RECEIVE_BODY;
			Driver_USART1.Receive(in_buffer+PACKET_HEADER_SIZE, size);
		}
		else
		{
			Packet packet = PacketManager_parse(in_buffer);

			if(packet->type == ERROR_PACKET)
			{
				if(packet->data[0] == PACKET_CRC_ERROR)
				{
					system_error("CRC error");
				}
				else if(packet->data[0] == PACKET_TYPES_ERROR)
				{
					system_error("Unknown packet type");
				}
			}
			else if(packet->type != LOAD_NET_INFO_PACKET)
			{
					system_error("Wrong packet type. NOT UNKNOWN. WRONG.");
			}
			else
			{
				EspUpdater_LoadNetworkData(packet);
				PacketManager_free(packet);

				state = STATE_RECEIVE_HEADER;
				Driver_USART1.Receive(in_buffer, PACKET_HEADER_SIZE);
			}
		}
	}

	if(event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		if(state == STATE_RECEIVE_BODY)
		{
			critical_error(SYSTEM_ERROR_IO, "Idle line while trying to "
					"receive body of EspUpdater packet");
		}
		else
		{
			LOGGING_Error("Idle line in RECEIVE_HEADER state in EspUpdater");
		}
	}
}


void EspUpdater_Init(uint32_t baudrate)
{
	Driver_USART1.Initialize(usart_callback);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
						  ARM_USART_DATA_BITS_8 |
						  ARM_USART_STOP_BITS_1 |
						  ARM_USART_PARITY_NONE |
						  ARM_USART_FLOW_CONTROL_NONE, baudrate);

	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
	Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);

	NVIC_EnableIRQ(EspUpdater_USART_IRQn);

	state = STATE_RECEIVE_HEADER;
	Driver_USART1.Receive(in_buffer, PACKET_HEADER_SIZE);
	USART1->CR1 &= ~(USART_CR1_IDLEIE);
}


void EspUpdater_LoadNetworkData(Packet data_packet)
{
	/*
	 * 1 byte for packet size
	 * 1 byte for ssid len
	 * 1 byte for pwd len
	 * 1 null character byte
	 */

	ESP8266_SendPacket(data_packet);
}

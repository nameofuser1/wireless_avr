/*
 * EspUpdater.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */


#include "EspUpdater.h"
#include "PacketManager.h"
#include "protocol.h"
#include "system.h"

#include <USART_STM32F10x.h>

/* Maximum buffer size */
#define INPUT_BUFFER_SIZE	50

static uint8_t in_buffer[INPUT_BUFFER_SIZE];

/* Reader state variables */
#define STATE_RECEIVE_HEADER 	0
#define STATE_RECEIVE_BODY		1

static uint8_t state = STATE_RECEIVE_HEADER;

/* Usart driver is set from out */
static ARM_DRIVER_USART driver;


static void usart_callback(uint32_t event)
{
	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		if(state == STATE_RECEIVE_HEADER)
		{
			uint16_t size = 0;
			for(uint8_t i=0; i<SIZE_FIELD_SIZE; i++)
			{
				size |= (in_buffer[SIZE_FIELD_OFFSET+i] >> 8*(SIZE_FIELD_SIZE-i - 1)) &
						0xFF;
			}

			driver->Receive(in_buffer+PACKET_HEADER_SIZE, size);
		}
		else
		{
			Packet packet = PacketManager_parse(in_buffer);

			if(packet->type == ERROR_PACKET)
			{
				if(packet->data[0] == PACKET_CRC_ERROR)
				{
					critical_error(SYSTEM_ERROR, "CRC error");
				}
				else if(packet->data[0] == PACKET_TYPE_ERROR)
				{
					critical_error(SYSTEM_ERROR, "Unknown packet type");
				}
			}
			else if(packet->type != LOAD_NET_INFO_PACKET)
			{
					critical_error(SYSTEM_ERROR,
							"Wrong packet type. NOT UNKNOWN. WRONG.");
			}
			else
			{
				EspUpdater_LoadNetworkData(packet);
				PacketManager_free(packet);
			}
		}
	}

	if(event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		if(state == STATE_RECEIVE_BODY)
		{
			critical_error(SYSTEM_ERROR, "Idle line while trying to "
					"receive body of EspUpdater packet");
		}
		else
		{
			critical_error(SYSTEM_ERROR, "Idle line in RECEIVE_HEADER "
					"state in EspUpdater");
		}
	}
}


void EspUpdater_init(ARM_DRIVER_USART drv, uint32_t baudrate)
{
	driver = drv;
	driver->Initialize(usart_callback);
	driver->Control(ARM_USART_CONTROL_RX | ARM_USART_CONTROL_TX, 1);
	driver->Control(ARM_USART_MODE_ASYNCHRONOUS, baudrate);
	driver->PowerControl(ARM_POWER_FULL);
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

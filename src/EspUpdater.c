/*
 * EspUpdater.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */


#include "EspUpdater.h"
#include "PacketManager.h"

#include <USART_STM32F10x.h>

static ARM_DRIVER_USART driver;

static void usart_callback(uint32_t event)
{
	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{

	}
}


void EspUpdater_init(ARM_DRIVER_USART drv, uint32_t baudrate)
{
	driver = drv;
	driver->Initialize(usart_callback);
	driver->Control(ARM_USART_CONTROL_RX | ARM_USART_CONTROL_TX, 1);
	driver->Control(ARM_USART_MODE_ASYNCHRONOUS, baudrate);
}


void EspUpdater_LoadNetworkData(void)
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
		packet->type = CMD_PACKET;
		packet->data = malloc(sizeof(uint8_t)*available);
		packet->data_length = available;

		uint8_t k = 0;
		while(available-- > 0)
		{
			packet->data[k++] = USART1_read();
		}

		//ESP_INFO_LOAD_GPIO_ENABLE();
		ESP8266_SendPacket(packet);

		free(packet->data);
	}
}

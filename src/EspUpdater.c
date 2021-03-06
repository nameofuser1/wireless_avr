/*
 * EspUpdater.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#include <stm32f10x.h>
#include <string.h>
#include <USART_STM32F10x.h>

#include "system/system.h"
#include "system/err.h"
#include "protocol.h"

#include "EspUpdater.h"
#include "esp8266.h"
#include "PacketManager.h"
#include "common/logging.h"


/* Maximum buffer size */
#define INPUT_BUFFER_SIZE	50

static uint8_t in_buffer[INPUT_BUFFER_SIZE];

/* Reader state variables */
#define STATE_RECEIVE_HEADER 	0
#define STATE_RECEIVE_BODY		1

/* Current state of module */
static uint8_t state = STATE_RECEIVE_HEADER;

/* Usart used to receive information */
#define EspUpdater_USART_IRQn	USART1_IRQn
extern ARM_DRIVER_USART 		Driver_USART1;


static void __recieve_header(void);
static void __recieve_body(uint32_t body_len);


extern uint32_t device_err;


static void usart_callback(uint32_t event)
{
	if(event & ARM_USART_EVENT_RX_OVERFLOW)
	{
		device_err = DEVICE_RX_OVERFLOW_ERROR;
		return;
	}

	if(event & ARM_USART_EVENT_TX_UNDERFLOW)
	{
		device_err = DEVICE_TX_UNDERFLOW_ERROR;
		return;
	}


	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{

		if(state == STATE_RECEIVE_HEADER)
		{
			uint32_t size = get_size_from_header(in_buffer) - PACKET_HEADER_SIZE;
			__recieve_body(size);
		}
		else
		{
			Packet packet = PacketManager_parse(in_buffer);

			if(packet->type != LOAD_NET_INFO_PACKET)
			{
				device_err = DEVICE_TYPES_ERROR;
				return;
			}
			else
			{
				EspUpdater_LoadNetworkData(packet);
				PacketManager_free(packet);
				LOGGING_Info("Loaded");
			}

			__recieve_header();
		}
	}

	if(event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		if(state == STATE_RECEIVE_BODY)
		{
			device_err = DEVICE_BODY_IDLE_LINE_ERROR;
		}
		else
		{
			device_err = DEVICE_HEADER_IDLE_LINE_ERROR;
		}
	}
}


void EspUpdater_Init(uint32_t baudrate)
{
	Driver_USART1.Initialize(usart_callback);
	Driver_USART1.PowerControl(ARM_POWER_FULL);

	int status = Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
						  	  	  	   ARM_USART_DATA_BITS_8 |
									   ARM_USART_STOP_BITS_1 |
									   ARM_USART_PARITY_NONE |
									   ARM_USART_FLOW_CONTROL_NONE, baudrate);

	if(status != ARM_DRIVER_OK)
	{
		LOGGING_Error("Can not initialize usart 1");
	}

	if(Driver_USART1.Control(ARM_USART_CONTROL_RX, 1) != ARM_DRIVER_OK)
	{
		LOGGING_Error("Can not initialize usart1 RX");
	}

	if(Driver_USART1.Control(ARM_USART_CONTROL_TX, 1) != ARM_DRIVER_OK)
	{
		LOGGING_Error("Can not initialize usart1 TX");
	}

	// It should be unnecessary
	NVIC_EnableIRQ(EspUpdater_USART_IRQn);
	__recieve_header();
	LOGGING_Info("EspUpdater started receiving headers");
}


void EspUpdater_DeInit(void)
{
	Driver_USART1.Control(ARM_USART_EVENT_RX_BREAK, 1);
	Driver_USART1.PowerControl(ARM_POWER_OFF);
	Driver_USART1.Uninitialize();
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


static void __recieve_body(uint32_t body_len)
{
	state = STATE_RECEIVE_BODY;

	Driver_USART1.Receive(in_buffer+PACKET_HEADER_SIZE, body_len);
	USART1->CR1 &= ~(USART_CR1_IDLEIE);
}


static void __recieve_header(void)
{
	state = STATE_RECEIVE_HEADER;

	Driver_USART1.Receive(in_buffer, PACKET_HEADER_SIZE);
	USART1->CR1 &= ~(USART_CR1_IDLEIE);
}

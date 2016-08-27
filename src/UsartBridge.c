/*
 * UsartBridge.c
 *
 *  Created on: Aug 26, 2016
 *      Author: kript0n
 */


#include "system.h"
#include "UsartBridge.h"
#include "USART_STM32F10x.h"
#include "esp8266.h"
#include "common/CircularBuffer.h"


#define FRAME_SIZE_MS 		25
#define UsartBridge_Driver 	Driver_USART2
#define UsartBridge_IRQn	USART2_IRQn

/* Usart driver and buffer for incoming data */
extern ARM_DRIVER_USART UsartBridge_Driver;
static uint8_t 			*bridge_buffer;
static uint32_t			bridge_buffer_size = 0;

/* Send buffer and its' size */
#define SEND_BUFFER_SIZE		10
static CircularBuffer 			send_buffer;

/* Here we use Packet instead of PacketBuffer
 * because we don't need to call Packet2Buf
 * as we need to send only packet->data not packet itself */
static Packet					processing_packet;

/* Static method definitions */
static void _send_packet(Packet packet);



static void usart_bridge_callback(uint32_t event)
{
	if((event & ARM_USART_EVENT_RECEIVE_COMPLETE) || (event & ARM_USART_EVENT_RX_TIMEOUT))
	{
		/* Create and send packet */
		uint32_t bytes_read = UsartBridge_Driver.GetRxCount();
		Packet usart_packet = PacketManager_CreatePacket(bridge_buffer,
				bytes_read, USART_PACKET);

		ESP8266_SendPacket(usart_packet);
		PacketManager_free(usart_packet);

		/* Start new receive */
		UsartBridge_Driver.Receive(bridge_buffer, bridge_buffer_size);
	}

	if(event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		PacketManager_free(processing_packet);

		if(!CircularBuffer_is_empty(&send_buffer))
		{
			processing_packet = (Packet)CircularBuffer_get(&send_buffer);
			_send_packet(processing_packet);
		}
	}
}


void UsartBridge_Init(uint32_t baudrate)
{
	UsartBridge_Driver.Initialize(usart_bridge_callback);
	UsartBridge_Driver.Control(ARM_USART_CONTROL_RX | ARM_USART_CONTROL_TX, 1);
	UsartBridge_Driver.Control(ARM_USART_MODE_ASYNCHRONOUS, 1);
	UsartBridge_Driver.PowerControl(ARM_POWER_FULL);

	/* FRAME_SIZE_MS milliseconds of frame in bytes */
	bridge_buffer_size = (baudrate * FRAME_SIZE_MS) / 1000 / 8;
	bridge_buffer = (uint8_t*)sys_malloc(sizeof(uint8_t) * bridge_buffer_size);

	/* Allocate buffer for sending packets */
	CircularBuffer_alloc(&send_buffer, SEND_BUFFER_SIZE);
}


void UsartBridge_DeInit(void)
{
	UsartBridge_Driver.Uninitialize();
}


void UsartBridge_Start(void)
{
	/* Start receiving */
	UsartBridge_Driver.Receive(bridge_buffer, bridge_buffer_size);
}


void UsartBridge_Stop(void)
{
	/* Abort receiving and sending */
	UsartBridge_Driver.Control(ARM_USART_ABORT_TRANSFER, 1);
}


void UsartBridge_Send(Packet usart_packet)
{
	Packet packet_cpy = PacketManager_Copy(usart_packet);

	if(CircularBuffer_is_empty(&send_buffer))
	{
		processing_packet = packet_cpy;
		_send_packet(processing_packet);
	}
	else
	{
		/* Disable IRQ for safe update */
		NVIC_DisableIRQ(UsartBridge_IRQn);
		CircularBuffer_put(&send_buffer, (void*)packet_cpy);
		NVIC_EnableIRQ(UsartBridge_IRQn);
	}
}


static void _send_packet(Packet packet)
{
	if(UsartBridge_Driver.Send(packet->data,
			packet->data_length) != ARM_DRIVER_OK)
	{
		io_error("Can't send data over UsartBridge");
	}
}

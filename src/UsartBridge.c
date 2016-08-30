/*
 * UsartBridge.c
 *
 *  Created on: Aug 26, 2016
 *      Author: kript0n
 */


#include "system.h"
#include "protocol.h"
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
static void		_send_packet(Packet packet);
static uint32_t _set_baudrate(uint8_t baudrate_byte);
static void 	_set_parity(uint8_t parity_byte);
static void 	_set_data_bits(uint8_t data_bits_byte);
static void 	_set_stop_bits(uint8_t stop_bits_byte);

/* Variables for tracking state */
static uint8_t initialized = 0;
static uint8_t started = 0;


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

	if(event & ARM_USART_EVENT_RX_FRAMING_ERROR)
	{
		ESP8266_SendError(USART_FRAME_ERROR_BYTE);
	}

	if(event & ARM_USART_EVENT_RX_PARITY_ERROR)
	{
		ESP8266_SendError(USART_PARITY_ERROR_BYTE);
	}
}


void UsartBridge_Init(Packet usart_config)
{
	if(!initialized)
	{
		UsartBridge_Driver.Initialize(usart_bridge_callback);
		UsartBridge_Driver.PowerControl(ARM_POWER_FULL);

		uint32_t baudrate = _set_baudrate(usart_config->data[USART_BAUDRATE_BYTE_OFFSET]);
		_set_parity(usart_config->data[USART_PARITY_BYTE_OFFSET]);
		_set_data_bits(usart_config->data[USART_DATA_BITS_BYTE_OFFSET]);
		_set_stop_bits(usart_config->data[USART_STOP_BITS_BYTE_OFFSET]);

		UsartBridge_Driver.Control(ARM_USART_CONTROL_RX, 1);
		UsartBridge_Driver.Control(ARM_USART_CONTROL_TX, 1);

		/* FRAME_SIZE_MS milliseconds of frame in bytes */
		bridge_buffer_size = (baudrate * FRAME_SIZE_MS) / 1000 / 8;
		bridge_buffer = (uint8_t*)sys_malloc(sizeof(uint8_t) * bridge_buffer_size);

		/* Allocate buffer for sending packets */
		CircularBuffer_alloc(&send_buffer, SEND_BUFFER_SIZE);


		initialized = 1;
	}
}


void UsartBridge_DeInit(void)
{
	if(initialized)
	{
		UsartBridge_Driver.Uninitialize();
		initialized = 0;
	}
}


void UsartBridge_Start(void)
{
	/* Start receiving */
	if(!started)
	{
		UsartBridge_Driver.Receive(bridge_buffer, bridge_buffer_size);
		started = 1;
	}
}


void UsartBridge_Stop(void)
{
	/* Abort receiving and sending */
	if(started)
	{
		UsartBridge_Driver.Control(ARM_USART_ABORT_TRANSFER, 1);
		started = 0;
	}
}


void UsartBridge_Send(Packet usart_packet)
{
	if(!initialized)
	{
		io_error("Can't send over USART. UsartDriver is not initialized");
	}

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


static uint32_t _set_baudrate(uint8_t baudrate_byte)
{
	uint32_t baudrate = 0;

	switch(baudrate_byte)
	{
		case USART_BAUDRATE_9600:
			baudrate = 9600;
			break;

		case USART_BAUDRATE_19200:
			baudrate = 19200;
			break;

		case USART_BAUDRATE_115200:
			baudrate = 115200;
			break;

		default:
			system_error("Wrong USART baudrate byte");
	}

	if(UsartBridge_Driver.Control(ARM_USART_MODE_ASYNCHRONOUS, baudrate) ==
			ARM_USART_ERROR_MODE)
	{
		io_error("Can't set USART mode");
	}

	return baudrate;
}


static void _set_parity(uint8_t parity_byte)
{
	uint32_t parity = 0;

	switch(parity_byte)
	{
		case USART_PARITY_NONE:
			parity = ARM_USART_PARITY_NONE;
			break;

		case USART_PARITY_EVEN:
			parity = ARM_USART_PARITY_EVEN;
			break;

		case USART_PARITY_ODD:
			parity = ARM_USART_PARITY_ODD;
			break;

		default:
			io_error("Wrong parity byte");
	}

	if(UsartBridge_Driver.Control(parity, 1) == ARM_USART_ERROR_PARITY)
	{
		io_error("Can't set USART parity");
	}
}


static void _set_data_bits(uint8_t data_bits_byte)
{
	uint32_t data_bits = 0;

	switch(data_bits_byte)
	{
		case USART_DATA_BITS_8:
			data_bits = ARM_USART_DATA_BITS_8;
			break;

		case USART_DATA_BITS_9:
			data_bits = ARM_USART_DATA_BITS_9;
			break;

		default:
			system_error("Wrong data_bits byte");
	}

	if(UsartBridge_Driver.Control(data_bits, 1) == ARM_USART_ERROR_DATA_BITS)
	{
		system_error("Can't set given data_bits");
	}
}


static void _set_stop_bits(uint8_t stop_bits_byte)
{
	uint32_t stop_bits = 0;

	switch(stop_bits_byte)
	{
		case USART_STOP_BITS_1:
			stop_bits = ARM_USART_STOP_BITS_1;
			break;

		case USART_STOP_BITS_2:
			stop_bits = ARM_USART_STOP_BITS_2;
			break;

		default:
			system_error("Wrong stop_bits byte");
	}

	if(UsartBridge_Driver.Control(stop_bits, 1) == ARM_USART_ERROR_STOP_BITS)
	{
		system_error("Can't set given stop bits");
	}
}














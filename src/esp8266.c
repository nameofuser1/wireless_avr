/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: Kamnev Yuriy
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>

#include "esp8266.h"
#include "protocol.h"
#include "transport.h"
#include "PacketManager.h"
#include "common/logging.h"
#include "common/CircularBuffer.h"
#include <Driver_USART.h>
#include <system/err.h>
#include <system/system.h>



/*
 * TODO:
 *  1.	Add LoadNetwork function.
 * 	2.	Add busy flag to prevent sending new packet
 * 			while updating network info.
 *
 *
 */

/*
 * GPIO Pin which is pulled up after ESP
 * connects to a network
 */
#define ESP_STATUS_GPIO_PORT 			GPIOB
#define ESP_STATUS_GPIO_PIN				GPIO_Pin_4
#define ESP_STATUS_GPIO_IDR				GPIO_IDR_IDR4
#define ESP_STATUS_GPIO_READ()			(ESP_STATUS_GPIO_PORT->IDR & ESP_STATUS_GPIO_IDR)

#define ESP_STATUS_GPIO_EXTI_PORT_SRC	GPIO_PortSourceGPIOB
#define ESP_STATUS_GPIO_EXTI_PIN_SRC	GPIO_PinSource4
#define ESP_STATUS_GPIO_EXTI_LINE		EXTI_Line4
#define ESP_STATUS_GPIO_EXTI_IRQn		EXTI4_IRQn


/* It is better to use IO interrupts
 * 	to turn on network info update mode.
 *
 * 	TODO:
 * 		Update ESP using interrupts  */
#define ESP_INFO_LOAD_GPIO				GPIOB
#define ESP_INFO_LOAD_GPIO_BS			GPIO_BSRR_BS5
#define ESP_INFO_LOAD_GPIO_BR			GPIO_BSRR_BR5
#define ESP_INFO_LOAD_GPIO_ENABLE()		(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BS)
#define ESP_INFO_LOAD_GPIO_DISABLE()	(ESP_INFO_LOAD_GPIO->BSRR |= ESP_INFO_LOAD_GPIO_BR)

/* USART which is connected to ESP */
#define ESP_USART_Typedef	USART3
#define ESP_USART_IRQn		USART3_IRQn
#define ESP_Driver_Usart 	Driver_USART3

extern ARM_DRIVER_USART 	ESP_Driver_Usart;

/* USART CONFIG */
#define ESP_USART_BAUDRATE 	115200

/* Buffer sizes for saving packets */
#define OUTCOME_PACKETS_BUFFER_SIZE 	10
#define INCOME_PACKETS_BUFFER_SIZE  	5

/* Save outcome packets here */
static CircularBuffer 	outcome_packets_buffer;
static PacketBuffer 	processing_packet;

/* Save income packets here */
static CircularBuffer 	income_packets_buffer;

/* Buffers for reading */
#define BODY_BUFFER_SIZE 	512

static uint8_t in_buffer[BODY_BUFFER_SIZE];

/* State defines */
typedef enum 	{ESP_STATE_RECV_HEADERS, ESP_STATE_RECV_BODY} EspState;
static EspState esp_state = ESP_STATE_RECV_HEADERS;

/* Set to 1 when ESP is connected to the network */
static uint8_t	ready = 0;

/* Import global error flag */
extern uint32_t device_err;

/* Static methods */
static void receive_header(void);
static void receive_body(uint32_t body_len);
static void _send_packet(PacketBuffer packet);
static void _free_packet_buf(PacketBuffer buf);



/*
 * Takes care about parsing packets from ESP.
 *
 * TODO:
 * 	Add one more state for updating ESP network info
 * 		and check for the error.
 */
static void usart_event_handler(uint32_t event)
{
	if(event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		LOGGING_Info("Packet is sent");
		_free_packet_buf(processing_packet);

		if(!CircularBuffer_is_empty(&outcome_packets_buffer))
		{
			processing_packet = (PacketBuffer)CircularBuffer_get(&outcome_packets_buffer);
			_send_packet(processing_packet);
		}
	}


	if(event & ARM_USART_EVENT_TX_UNDERFLOW)
	{
		device_err = DEVICE_TX_UNDERFLOW_ERROR;
		return;
	}

	/*
	 * Does not matter what happened while reading
	 * if ESP is not ready */
	if(!ready)
	{
		return;
	}

	if(event & ARM_USART_EVENT_RX_OVERFLOW)
	{
		device_err = DEVICE_RX_OVERFLOW_ERROR;
		return;
	}


	if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		if(esp_state == ESP_STATE_RECV_HEADERS)
		{
			uint32_t packet_size = get_size_from_header(in_buffer) - PACKET_HEADER_SIZE;
			receive_body(packet_size);
			esp_state = ESP_STATE_RECV_BODY;
		}
		else
		{
			Packet packet = PacketManager_parse(in_buffer);
			CircularBuffer_put(&income_packets_buffer, (void*)packet);

			esp_state = ESP_STATE_RECV_HEADERS;
			receive_header();
		}
	}
	else if(event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		if(esp_state == ESP_STATE_RECV_HEADERS)
		{
			device_err = DEVICE_HEADER_IDLE_LINE_ERROR;
			return;
		}
		else
		{
			if(ESP_Driver_Usart.GetStatus().rx_busy == 1)
			{
				device_err = DEVICE_BODY_IDLE_LINE_ERROR;
				return;
			}
		}
	}

	if(event & ARM_USART_EVENT_RX_FRAMING_ERROR)
	{
		device_err = DEVICE_FRAMING_ERROR;
		return;
	}

	if(event & ARM_USART_EVENT_RX_PARITY_ERROR)
	{
		device_err = DEVICE_PARITY_ERROR;
		return;
	}

}


/*
 * Initialize UART
 */
void ESP8266_Init(void)
{
	ESP_Driver_Usart.Initialize((ARM_USART_SignalEvent_t)usart_event_handler);
	ESP_Driver_Usart.PowerControl(ARM_POWER_FULL);
	ESP_Driver_Usart.Control(ARM_USART_MODE_ASYNCHRONOUS |
							 ARM_USART_PARITY_NONE |
							 ARM_USART_DATA_BITS_8 |
							 ARM_USART_STOP_BITS_1 |
							 ARM_USART_FLOW_CONTROL_NONE, ESP_USART_BAUDRATE);

	ESP_Driver_Usart.Control(ARM_USART_CONTROL_TX, 1);
	ESP_Driver_Usart.Control(ARM_USART_CONTROL_RX, 0);

	// It should not be necessary
	NVIC_EnableIRQ(ESP_USART_IRQn);

	if(!CircularBuffer_alloc(&outcome_packets_buffer, OUTCOME_PACKETS_BUFFER_SIZE))
	{
		memory_error( "Can't allocate outcome buffer");
	}

	if(!CircularBuffer_alloc(&income_packets_buffer, INCOME_PACKETS_BUFFER_SIZE))
	{
		memory_error( "Can't allocate income buffer");
	}

	PacketManager_init();

	/* Initialize status GPIO as input with pull down */
	GPIO_InitTypeDef status_gpio;
	status_gpio.GPIO_Mode = GPIO_Mode_IPD;
	status_gpio.GPIO_Pin = ESP_STATUS_GPIO_PIN;
	GPIO_Init(ESP_STATUS_GPIO_PORT, &status_gpio);

	/* Configure EXTI line */
	GPIO_EXTILineConfig(ESP_STATUS_GPIO_EXTI_PORT_SRC, ESP_STATUS_GPIO_EXTI_PIN_SRC);
	NVIC_EnableIRQ(ESP_STATUS_GPIO_EXTI_IRQn);

	/* Enable EXTI interrupt on status pin */
	EXTI_InitTypeDef status_exti;
	status_exti.EXTI_Line = ESP_STATUS_GPIO_EXTI_LINE;
	status_exti.EXTI_Mode = EXTI_Mode_Interrupt;
	status_exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	status_exti.EXTI_LineCmd = ENABLE;

	EXTI_Init(&status_exti);

	ready = ESP_STATUS_GPIO_READ();
	if(ready)
	{
		ESP_Driver_Usart.Control(ARM_USART_CONTROL_RX, 1);
		receive_header();
	}
}


void ESP8266_Start(void)
{
	receive_header();
}


void ESP8266_DeInit(void)
{
	ESP_Driver_Usart.Uninitialize();

	CircularBuffer_free(&outcome_packets_buffer, true);
	CircularBuffer_free(&income_packets_buffer, true);
}


/*
 * When ESP is connected to a network it pulls
 * 	pin up. After that we can safely read data from it.
 */
void ESP8266_WaitForReady(void)
{
	while(!ready);
}


bool ESP8266_Ready(void)
{
	return ready;
}


bool ESP8266_SendPacket(Packet packet)
{
	if(packet->type != NONE_PACKET)
	{
		uint32_t buf_len = 0;

		PacketBuffer packet_buf = sys_malloc(sizeof(struct _packet_buffer));
		//LOGGING_Debug("Creating packet buffer");
		packet_buf->buf = PacketManager_Packet2Buf(packet, &buf_len);
		//LOGGING_Debug("Created packet buffer");
		packet_buf->length = buf_len;

		if(CircularBuffer_is_empty(&outcome_packets_buffer))
		{
			processing_packet = packet_buf;
			_send_packet(processing_packet);
		}
		else
		{
			/* Disable IRQ for safe update */
			NVIC_DisableIRQ(ESP_USART_IRQn);
			CircularBuffer_put(&outcome_packets_buffer, (void*)(packet_buf));
			NVIC_EnableIRQ(ESP_USART_IRQn);
		}

		return true;

		LOGGING_Debug("Sent pack");
	}

	return false;
}


bool ESP8266_SendAck(void)
{
	Packet ack_packet = PacketManager_CreatePacket(NULL, 0, ACK_PACKET, TRUE);
	bool res = ESP8266_SendPacket(ack_packet);
	PacketManager_free(ack_packet);

	return res;
}


bool ESP8266_SendError(uint8_t error)
{
	uint8_t err[1] = {error};
	Packet err_packet = PacketManager_CreatePacket(err, 1, ERROR_PACKET, TRUE);
	bool res = ESP8266_SendPacket(err_packet);

	PacketManager_free(err_packet);

	return res;
}


bool ESP8266_TransmissionStatus(void)
{
	return (ESP_Driver_Usart.GetStatus().tx_busy == 1);
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

	return PacketManager_CreatePacket(NULL, 0, NONE_PACKET, TRUE);
}


/*
 * Receiving header.
 * Disable IDLE line interrupt so we can wait forever.
 *
 * TODO:
 * 		Replace system error with device error
 */
static void receive_header(void)
{
	memset(in_buffer, '0', BODY_BUFFER_SIZE);
	if(ESP_Driver_Usart.Receive(in_buffer, PACKET_HEADER_SIZE) != ARM_DRIVER_OK)
	{
		system_error("Can't receive packet headers");
	}

	ESP_USART_Typedef->CR1 &= ~(USART_CR1_IDLEIE);
	LOGGING_Info("ESP Receiving header is started");
}


/*
 * Receive packet body
 */
static void receive_body(uint32_t body_len)
{
	int32_t status = ESP_Driver_Usart.Receive(in_buffer+PACKET_HEADER_SIZE, body_len);

	switch(status)
	{
		case ARM_DRIVER_ERROR_BUSY:
			device_err = DEVICE_RECEIVE_BUSY_ERROR;
			break;

		case ARM_DRIVER_ERROR_PARAMETER:
			device_err = DEVICE_RECEIVE_PARAMETER_ERROR;
			break;

		case ARM_DRIVER_ERROR:
			device_err = DEVICE_UNKNOWN_DRIVER_ERROR;
			break;
	}
	ESP_USART_Typedef->CR1 &= ~(USART_CR1_IDLEIE);
}


static void _send_packet(PacketBuffer packet)
{
	int32_t status = ESP_Driver_Usart.Send((void*)(packet->buf), packet->length);

	switch(status)
	{
		case ARM_DRIVER_ERROR_BUSY:
			device_err = DEVICE_SEND_BUSY_ERROR;
			break;

		case ARM_DRIVER_ERROR_PARAMETER:
			device_err = DEVICE_SEND_PARAMETER_ERROR;
			break;

		case ARM_DRIVER_ERROR:
			device_err = DEVICE_UNKNOWN_DRIVER_ERROR;
			break;
	}
}


static void _free_packet_buf(PacketBuffer buf)
{
	free(buf->buf);
	free(buf);
}


/*
 *
 */
void EXTI4_IRQHandler(void)
{
	uint8_t rising = GPIO_ReadInputDataBit(ESP_STATUS_GPIO_PORT, ESP_STATUS_GPIO_PIN);

	if(rising)
	{
		ESP_Driver_Usart.Control(ARM_USART_CONTROL_RX, 1);
		receive_header();
		ready = 1;
	}
	else
	{
		ESP_Driver_Usart.Control(ARM_USART_ABORT_RECEIVE, 1);
		ESP_Driver_Usart.Control(ARM_USART_CONTROL_RX, 0);
		ready = 0;
	}

	EXTI_ClearITPendingBit(ESP_STATUS_GPIO_EXTI_LINE);
}

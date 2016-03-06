/*
 * esp8266.c
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */
#include "esp8266.h"
#include "periph/usart.h"
#include "periph/usart3.h"
#include "avr_flasher.h"

#define ESP_USART USART3

bool ESP8266_SendData(uint8_t *data, uint8_t len)
{
	USART3_tx_array(data, len);
	return true;
}


bool ESP8266_SendAck(void)
{
	uint8_t ack[4] = {ACK_PACKET_BYTE, ACK_PACKET_BYTE, ACK_PACKET_BYTE, ACK_PACKET_BYTE};
	USART3_tx_array(ack, 4);
	return true;
}


bool ESP8266_SendError(void)
{
	uint8_t err[4] = {ERROR_PACKET_BYTE, ERROR_PACKET_BYTE, ERROR_PACKET_BYTE, ERROR_PACKET_BYTE};
	USART3_tx_array(err, 4);
	return true;
}


uint32_t ESP8266_available(void)
{
	return USART3_available();
}


uint8_t ESP8266_read(void)
{
	return USART3_read();
}



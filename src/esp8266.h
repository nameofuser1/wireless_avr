/*
 * esp8266.h
 *
 *  Created on: 17 февр. 2016 г.
 *      Author: kripton
 */

#ifndef ESP8266_H_
#define ESP8266_H_

#include "stm32f10x.h"
#include "PacketManager.h"
#include <stdbool.h>

void ESP8266_init(void);
void ESP8266_DeInit(void);

/*
 * Loop until status GPIO is set
 */
void ESP8266_WaitForReady(void);
bool ESP8266_Ready(void);

bool ESP8266_SendPacket(Packet packet);
bool ESP8266_SendAck(void);
bool ESP8266_SendError(uint8_t error);

/*
 * If we have wrong packet we ask for it again.
 */
bool ESP8266_SendLastPacket(void);

/*
 * Return true is transmission is still pending
 * And false otherwise
 */
bool ESP8266_TransmissionStatus(void);

/*
 * Loads ssid info into ESP
 */
void 		ESP8266_LoadNetworkData(void);

uint32_t 	ESP8266_available(void);
uint8_t  	ESP8266_read(void);
void		ESP8266_flush_rx(void);
void		ESP8266_flush_tx(void);



#endif /* ESP8266_H_ */

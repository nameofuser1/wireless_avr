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

void 		ESP8266_Init(void);
void 		ESP8266_DeInit(void);
void 		ESP8266_WaitForReady(void);
bool 		ESP8266_Ready(void);
bool 		ESP8266_SendPacket(Packet packet);
bool 		ESP8266_SendAck(void);
bool 		ESP8266_SendError(uint8_t error);
bool 		ESP8266_TransmissionStatus(void);
bool	 	ESP8266_Available(void);
Packet 		ESP8266_GetPacket(void);



#endif /* ESP8266_H_ */

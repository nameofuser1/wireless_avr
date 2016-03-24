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

bool ESP8266_SendPacket(Packet packet);
bool ESP8266_SendAck(void);
bool ESP8266_SendError(uint8_t error);
bool ESP8266_LoadNetworkData(uint8_t *ssid, uint8_t ssid_len, uint8_t *pwd, uint8_t pwd_len);

uint32_t ESP8266_available(void);
uint8_t  ESP8266_read(void);



#endif /* ESP8266_H_ */

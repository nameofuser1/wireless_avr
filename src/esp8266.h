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

/*
 * Loop until status GPIO is set
 */
void ESP8266_WaitForReady(void);

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
bool ESP8266_LoadNetworkData(uint8_t *ssid, uint8_t ssid_len, uint8_t *pwd, uint8_t pwd_len);

uint32_t 	ESP8266_available(void);
uint8_t  	ESP8266_read(void);
void		ESP8266_flush_rx(void);



#endif /* ESP8266_H_ */

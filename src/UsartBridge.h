/*
 * UsartBridge.h
 *
 *  Created on: Aug 26, 2016
 *      Author: kript0n
 */

#ifndef USARTBRIDGE_H_
#define USARTBRIDGE_H_

#include "PacketManager.h"

void UsartBridge_Init(Packet usart_config);
void UsartBridge_DeInit(void);
void UsartBridge_Start(void);
void UsartBridge_Stop(void);
void UsartBridge_Send(Packet usart_packet);


#endif /* USARTBRIDGE_H_ */

/*
 * EspUpdater.h
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_ESPUPDATER_H_
#define SRC_ESPUPDATER_H_

#include "PacketManager.h"

/*
 * Loads ssid info into ESP
 */
void 		EspUpdater_LoadNetworkData(Packet data_packet);

#endif /* SRC_ESPUPDATER_H_ */

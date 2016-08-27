/*
 * transport.h
 *
 *  Created on: Aug 27, 2016
 *      Author: kript0n
 */

#ifndef SRC_TRANSPORT_H_
#define SRC_TRANSPORT_H_

typedef struct _packet_buffer {
	uint8_t *buf;
	uint32_t length;
} *PacketBuffer;

#endif /* SRC_TRANSPORT_H_ */

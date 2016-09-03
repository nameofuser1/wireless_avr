/*
 * err.h
 *
 *  Created on: 1 сент. 2016 г.
 *      Author: bigmac
 */

#ifndef SRC_ERR_H_
#define SRC_ERR_H_

#include <inttypes.h>

#define DEVICE_OK						0
#define DEVICE_CRC_ERROR 				1
#define DEVICE_MEMORY_ERROR 			2
#define DEVICE_LENGTH_ERROR				3
#define DEVICE_TYPES_ERROR				4
#define DEVICE_HEADER_IDLE_LINE_ERROR	5
#define DEVICE_BODY_IDLE_LINE_ERROR		6
#define DEVICE_RECEIVE_BUSY_ERROR		7
#define DEVICE_RECEIVE_PARAMETER_ERROR	8
#define DEVICE_SEND_BUSY_ERROR			9
#define DEVICE_SEND_PARAMETER_ERROR		10
#define DEVICE_UNKNOWN_DRIVER_ERROR		11
#define DEVICE_INITIALIZATION_ERROR		12
#define DEVICE_PROGRAMMER_TYPE_ERROR	13
#define DEVICE_PROGRAMMING_ERROR		14

extern uint32_t 	device_err;

#endif /* SRC_ERR_H_ */

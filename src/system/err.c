/*
 * err.c
 *
 *  Created on: 4 сент. 2016 г.
 *      Author: bigmac
 */

#include <stm32f10x.h>
#include <system/err.h>
#include <system/system.h>
#include "soft_timers/HardwareTimer.h"
#include "soft_timers/SoftwareTimer.h"

#define ERROR_NUM	(DEVICE_PARITY_ERROR + 1)

/*
 *	System error functions typedef
 */
typedef 	void (*system_error_func_t)(char *msg);

/*
 * Initialize global error flag
 */
uint32_t 	device_err = DEVICE_OK;

/*
 * Errors' log strings
 */
char *error_str[ERROR_NUM] =
{
		"Device ok", 	"Wrong crc", 	"Memory error", 	"Packet length error", 		"Packet type error",
		"Idle line on ESP rx line while receiving headers", "Idle line on ESP rx line while receiving body",
		"ESP Receive busy",		"ESP Receive parameter error", "ESP Send busy error", "ESP send parameter error",
		"ESP unknown driver error", "Error while initializing device", "Unsupported programmer type",
		"Programming error", 		"Rx overflow", 		"Tx underflow",			"Framing error", 		"Parity error"
};

/*
 * Call function on corresponding string from error_str
 */
system_error_func_t error_funcs[ERROR_NUM] =
{
		NULL, 		system_error, 	memory_error,	system_error,	system_error,
		io_error,	io_error,		io_error,		io_error,		io_error,
		io_error,	io_error,		system_error,	system_error,	system_error,
		io_error,	io_error,		io_error,		io_error

};


void handle_error(uint32_t error_code)
{
	error_funcs[error_code](error_str[error_code]);
}

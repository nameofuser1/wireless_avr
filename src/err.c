/*
 * err.c
 *
 *  Created on: 4 сент. 2016 г.
 *      Author: bigmac
 */

#include "err.h"

#include <stm32f10x.h>
#include "system.h"
#include "soft_timers/HardwareTimer.h"
#include "soft_timers/SoftwareTimer.h"

extern HardwareTimerDriver SystemTimer;
static SoftwareTimer err_timer;

static void handle_error(uint32_t error_byte);
static void err_timer_cb(void);

/*
 * Global error flag
 * Extern it and set error
 */
uint32_t 	device_err = DEVICE_OK;

void Err_init(void)
{
	SoftwareTimer_Init(&err_timer);
	SoftwareTimer_Add_cb(&err_timer, (SoftTimerCallback)err_timer_cb);
	SoftwareTimer_Arm(&err_timer, Timer_Repeat, 1);

	SystemTimer.AddTimer(&err_timer);
}


static void err_timer_cb(void)
{
	if(device_err != DEVICE_OK)
	{
		handle_error(device_err);
	}
}


static void handle_error(uint32_t error_byte)
{
	switch(error_byte)
	{
		case DEVICE_TYPES_ERROR:
			system_error("Unknown packet");
			break;

		case DEVICE_CRC_ERROR:
			system_error("Wrong crc");
			break;

		case DEVICE_HEADER_IDLE_LINE_ERROR:
			io_error("Idle line on ESP rx line while receiving headers");
			break;

		case DEVICE_BODY_IDLE_LINE_ERROR:
			io_error("Idle line on ESP rx line while receiving body");
			break;

		case DEVICE_RECEIVE_BUSY_ERROR:
			io_error("ESP Receive busy");
			break;

		case DEVICE_RECEIVE_PARAMETER_ERROR:
			io_error("ESP Receive parameter error");
			break;

		case DEVICE_SEND_BUSY_ERROR:
			io_error("ESP Send busy error");
			break;

		case DEVICE_SEND_PARAMETER_ERROR:
			io_error("ESP send parameter error");
			break;

		case DEVICE_UNKNOWN_DRIVER_ERROR:
			io_error("ESP unknown driver error");
			break;

		case DEVICE_MEMORY_ERROR:
			memory_error("ESP memory error");
			break;

		case DEVICE_LENGTH_ERROR:
			system_error("Packet length error");
			break;

		case DEVICE_PROGRAMMER_TYPE_ERROR:
			system_error("Unsupported programmer type");
			break;

		case DEVICE_INITIALIZATION_ERROR:
			system_error("Error while initializing device");
			break;

		case DEVICE_PROGRAMMING_ERROR:
			system_error("Programming error");
			break;

		default:
			system_error("Unknown error code");
	}
}

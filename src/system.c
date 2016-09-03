/*
 * system.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#include "system.h"
#include "common/logging.h"
#include <stdlib.h>
#include <stm32f10x.h>
#include <core_cm3.h>

#define SYSTEM_ERROR		0
#define SYSTEM_ERROR_MEM	1
#define SYSTEM_ERROR_IO		2
#define SYSTEM_ERRORS_NUM	3

static char* errors[SYSTEM_ERRORS_NUM] = {"System error", "Memory error", "IO error"};

/*
 * 	Restart controller
 */
static void critical_error(uint8_t err, char *msg)
{
	LOGGING_Error(errors[err]);

	if(msg != NULL)
	{
		LOGGING_Error(msg);
	}

	LOGGING_Error("Resetting device");
	/* Small delay for delivering LOG message */
	for(volatile uint32_t i=0; i<10000000; i++);
	NVIC_SystemReset();
}


void system_error(char *msg)
{
	critical_error(SYSTEM_ERROR, msg);
}


void memory_error(char *msg)
{
	critical_error(SYSTEM_ERROR_MEM, msg);
}


void io_error(char *msg)
{
	critical_error(SYSTEM_ERROR_IO, msg);
}


void* sys_malloc(int32_t size)
{
	void *arr = malloc(size);
	if(arr == NULL)
	{
		critical_error(SYSTEM_ERROR_MEM, NULL);
	}

	return arr;
}

/*
 * system.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#include "system.h"
#include "common/Logging.h"
#include <stdlib.h>
#include <core_cm3.h>

#define NULL (void*)0


static char* errors[SYSTEM_ERRORS_NUM] = {"System memory overflow"};

/*
 * 	Restart controller
 */
void critical_error(uint8_t err, char *msg)
{
	LOGGING_log(errors[err], LOG_ERROR);

	if(msg != NULL)
	{
		LOGGING_Log(msg, LOG_ERROR);
	}

	LOGGING_Log("Resetting device", LOG_ERROR);
	NVIC_SystemReset();
}


void* sys_malloc(size_t size)
{
	void *arr = malloc(size);
	if(arr == NULL)
	{
		critical_error(SYSTEM_ERROR_MEM, NULL);
	}

	return arr;
}

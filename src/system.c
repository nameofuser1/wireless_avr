/*
 * system.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */

#include "system.h"
#include "common/Logging.h"

#include <core_cm3.h>

/*
 * 	Restart controller
 */
void critical_error(char *msg)
{
	LOGGING_Log(msg, LOG_ERROR);
	LOGGING_Log("Resetting device", LOG_ERROR);
	NVIC_SystemReset();
}

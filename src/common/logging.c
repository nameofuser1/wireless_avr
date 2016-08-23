/*
 * logging.c
 *
 *  Created on: Aug 21, 2016
 *      Author: kript0n
 */

#include "logging.h"
#include <stdio.h>

static LogLevel level = LOG_INFO;


static const
char *levels_names[LOG_INFO+1] = {"LOG_ERROR", "LOG_DEBUG", "LOG_WARNING", "LOG_INFO"};


void LOGGING_Log(const char *message, LogLevel lvl)
{
	if(lvl > level)
	{
		printf("%s -- %s", levels_names[lvl], message);
	}
}


void LOGGING_SetLevel(LogLevel lvl)
{
	level = lvl;
}



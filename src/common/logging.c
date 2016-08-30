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


void LOGGING_Log(const char *message, const LogLevel lvl)
{
	if(lvl <= level)
	{
		printf("%s -- %s\r\n", levels_names[lvl], message);
	}
}


void LOGGING_Error(const char *msg)
{
	LOGGING_Log(msg, LOG_ERROR);
}


void LOGGING_Debug(const char *msg)
{
	LOGGING_Log(msg, LOG_DEBUG);
}


void LOGGING_Warning(const char *msg)
{
	LOGGING_Log(msg, LOG_WARNING);
}


void LOGGING_Info(const char *msg)
{
	LOGGING_Log(msg, LOG_INFO);
}


void LOGGING_SetLevel(LogLevel lvl)
{
	level = lvl;
}




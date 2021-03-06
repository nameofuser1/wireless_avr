/*
 * logging.c
 *
 *  Created on: Aug 21, 2016
 *      Author: kript0n
 */

#include "logging.h"
#include <stdio.h>
#include <stdarg.h>
#include <system/system.h>

#define __FORMAT_LOG __attribute__ ((format (printf, 1, 2)))


static LogLevel level = LOG_INFO;


static const
char *levels_names[LOG_INFO+1] = {"LOG_ERROR", "LOG_DEBUG", "LOG_WARNING", "LOG_INFO"};


static void LOGGING_VLog(const LogLevel lvl, const char *format, va_list args)
{
	char str[100];
	vsprintf(str, format, args);

	if(lvl <= level)
	{
		printf("%s -- %s\r\n", levels_names[lvl], str);
	}
}


void LOGGING_Log(const LogLevel lvl, const char *message)
{
	if(lvl <= level)
	{
		printf("%s -- %s\r\n", levels_names[lvl], message);
	}
}


void __FORMAT_LOG LOGGING_Error(const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	LOGGING_VLog(LOG_ERROR, msg, args);
	va_end(args);
}


void __FORMAT_LOG LOGGING_Debug(const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	LOGGING_VLog(LOG_DEBUG, msg, args);
	va_end(args);
}


void __FORMAT_LOG LOGGING_Warning(const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	LOGGING_VLog(LOG_WARNING, msg, args);
	va_end(args);
}


void __FORMAT_LOG LOGGING_Info(const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	LOGGING_VLog(LOG_INFO, msg, args);
	va_end(args);
}


void LOGGING_SetLevel(LogLevel lvl)
{
	level = lvl;
}


char* to_hex_str(uint8_t *buf, uint32_t len)
{
	const char *hex = "0123456789ABCDEF";
	const char del = ' ';
	char *out = (char*)sys_malloc(sizeof(char)*len*2 + 1);
	uint8_t* pin = buf;

	char * pout = out;
	for(; pin < buf+len; pout +=3, pin++)
	{
		pout[0] = hex[(*pin>>4) & 0xF];
		pout[1] = hex[ *pin     & 0xF];
		pout[2] = del;
	}

	pout[-1] = 0;

	return out;
}



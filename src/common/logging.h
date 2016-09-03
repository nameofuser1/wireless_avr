/*
 * logging.h
 *
 *  Created on: Aug 21, 2016
 *      Author: kript0n
 */

#ifndef SRC_LOGGING_H_
#define SRC_LOGGING_H_

#include <inttypes.h>

typedef enum {LOG_ERROR=0, LOG_DEBUG, LOG_WARNING, LOG_INFO} LogLevel;

void LOGGING_Log(const LogLevel lvl, const char *message);
void LOGGING_Error(const char *msg, ...);
void LOGGING_Debug(const char *msg, ...);
void LOGGING_Warning(const char *msg, ...);
void LOGGING_Info(const char *msg, ...);

void LOGGING_SetLevel(LogLevel lvl);

char* to_hex_str(uint8_t *buf, uint32_t len);


#endif /* SRC_LOGGING_H_ */

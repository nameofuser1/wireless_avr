/*
 * logging.h
 *
 *  Created on: Aug 21, 2016
 *      Author: kript0n
 */

#ifndef SRC_LOGGING_H_
#define SRC_LOGGING_H_

typedef enum {LOG_ERROR=0, LOG_DEBUG, LOG_WARNING, LOG_INFO} LogLevel;

void LOGGING_Log(char *message, LogLevel lvl);
void LOGGING_SetLevel(LogLevel lvl);



#endif /* SRC_LOGGING_H_ */

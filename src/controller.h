/*
 * controller.h
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define CONTROLLER_ACTION_NUM 	5
#define CONTROLLER_BUF_SIZE 	256
#define ACTION_TIMEOUT_MS		500

#define PROG_AVR_BYTE 		0x00
#define PROG_ARDUINO_BYTE	0x01
#define PROG_STM_BYTE		0x02

typedef enum { READY = 0, READ_MCU_INFO, READ_CMD, TERMINATE, FAILED} ProgramState;

typedef enum { NONE = 0, INITIAL_ERROR, PROG_TYPE_ERROR, IO_ERROR, PACKET_TYPE_ERROR } ResultCode;

typedef enum { PROG_AVR, PROG_ARDUINO, PROG_STM, PROG_NONE }	ProgrammerType;


void 			CONTROLLER_init(void);
void 			CONTROLLER_DeInit(void);
ResultCode 		CONTROLLER_perform_action(void);
ProgramState 	CONTROLLER_get_state(void);
void			CONTROLLER_clear_error(void);


#endif /* CONTROLLER_H_ */

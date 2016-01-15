/*
 * controller.h
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define CONTROLLER_ACTION_NUM 	4
#define CONTROLLER_BUF_SIZE 	256
#define ACTION_TIMEOUT_MS		2000

typedef enum { READY, READ_INIT, READ_CMD, SEND_CMD } ProgramState;
typedef enum { NONE = 0, INITIAL_ERROR, TIMEOUT} ResultCode;

void 			CONTROLLER_init(void);
ResultCode 		CONTROLLER_perform_action(void);
ProgramState 	CONTROLLER_get_state(void);
void			CONTROLLER_clear_error(void);


#endif /* CONTROLLER_H_ */

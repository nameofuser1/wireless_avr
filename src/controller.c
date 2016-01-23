/*
 * controller.c
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */
#include "controller.h"
#include "UART.h"
#include "avr_flasher.h"
#include "spi.h"
#include "soft_timers/SoftwareTimer2.h"

static void CONTROLLER_state_ready(void);
static void CONTROLLER_state_read_init(void);
static void CONTROLLER_state_read_cmd(void);
static void CONTROLLER_state_send_cmd(void);
static void CONTROLLER_state_terminate(void);
static void CONTROLLER_state_error(void);
static inline void send_ack(void);
static inline void send_error(void);
static inline void update_write_pointer(void);
static inline void update_read_pointer(void);

static ProgramState state = READY;
static ResultCode error = NONE;
static void (*actions[CONTROLLER_ACTION_NUM])(void);

static uint32_t read_pointer = 0;
static uint32_t write_pointer = 0;
static uint32_t remaining_data = 0;
static uint8_t data_buffer[CONTROLLER_BUF_SIZE];

static SoftwareTimer timeout_timer;
static SoftwareTimer wait_at_ready_timer;

void CONTROLLER_init(void)
{
	printf("Controller init\r\n");
	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//1 ms
	SoftwareTimer2_start();

	SoftwareTimer_init(&timeout_timer);
	SoftwareTimer_init(&wait_at_ready_timer);

	SPI_init(SPI1);
	SPI_enable(SPI1);

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_INIT] = CONTROLLER_state_read_init;
	actions[READ_CMD] = CONTROLLER_state_read_cmd;
	actions[SEND_CMD] = CONTROLLER_state_send_cmd;
	actions[TERMINATE] = CONTROLLER_state_terminate;
	actions[FAILED] = CONTROLLER_state_error;
}

ResultCode CONTROLLER_perform_action(void)
{
	(*actions[state])();

	return error;
}

ProgramState CONTROLLER_get_state(void)
{
	return state;
}


static void CONTROLLER_state_ready(void)
{
	if(remaining_data < 4)
	{
		if(!USART3_is_empty())
		{
			data_buffer[write_pointer] = USART3_read();
			printf("Got byte in state ready: 0x%02x\r\n", data_buffer[write_pointer]);
			update_write_pointer();
			remaining_data++;
		}
	}
	else
	{
		for(uint32_t i=0; i<4; i++)
		{
			if(data_buffer[read_pointer] != INIT_PACKET_BYTE)
			{
				error = INITIAL_ERROR;
				state = FAILED;
				return;
			}
			update_read_pointer();
			remaining_data--;
		}

		AVRFlasher_reset_enable();
		send_ack();

		SoftwareTimer_arm(&wait_at_ready_timer, OnePulse, 30);
		SoftwareTimer_start(&soft_timer2, &wait_at_ready_timer);
		SoftwareTimer_wait_for(&wait_at_ready_timer);

		SoftwareTimer_arm(&timeout_timer, OnePulse, ACTION_TIMEOUT_MS);
		SoftwareTimer_start(&soft_timer2, &timeout_timer);
		state = READ_CMD;
	}
}

static void CONTROLLER_state_read_init(void)
{

}

static void CONTROLLER_state_read_cmd(void)
{
	if(timeout_timer.state == Done)
	{
		error = TIMEOUT;
	}
	else
	{
		if(remaining_data < 4)
		{
			if(!USART3_is_empty())
			{
				data_buffer[write_pointer] = USART3_read();
				printf("Got byte in read cmd: 0x%02x\r\n", data_buffer[write_pointer]);
				update_write_pointer();
				remaining_data++;
			}
		}
		else
		{
			/*
			 * 	If at least one byte is not STOP_PACKET_BYTE
			 *	State will become SEND_CMD
			 */
			state = TERMINATE;

			for(uint32_t i = 0; i<4; i++)
			{
				if(data_buffer[read_pointer+i] != STOP_PACKET_BYTE)
				{
					SoftwareTimer_stop(&soft_timer2, &timeout_timer);
					//timeout_timer.state = Idle;
					state = SEND_CMD;
					break;
				}
			}
		}
	}
}

static void CONTROLLER_state_send_cmd(void)
{
	AvrCommand command;
	command.b1 = data_buffer[read_pointer];
	update_read_pointer();
	command.b2 = data_buffer[read_pointer];
	update_read_pointer();
	command.b3  = data_buffer[read_pointer];
	update_read_pointer();
	command.b4 = data_buffer[read_pointer];
	update_read_pointer();
	remaining_data -= 4;

	uint8_t res[4];
	AVRFlasher_send_command(&command, res);

	USART_SendArray(USART3, res, 4);
	state = READ_CMD;

	printf("Get avr answer: ");
	for(int i=0; i<4; i++) printf("0x%02x ", res[i]);
	printf("\r\n\r\n");
}


static void CONTROLLER_state_terminate(void)
{
	AVRFlasher_reset_disable();

	read_pointer = 0;
	write_pointer = 0;
	remaining_data = 0;

	SoftwareTimer_wait_for(&timeout_timer);
	timeout_timer.state = Idle;

	send_ack();
	state = READY;
}


static void CONTROLLER_state_error(void)
{
	AVRFlasher_reset_disable();

	read_pointer = 0;
	write_pointer = 0;
	remaining_data = 0;

	SoftwareTimer_wait_for(&timeout_timer);
	timeout_timer.state = Idle;

	error = NONE;
	send_error();
	state = READY;
}


void CONTROLLER_clear_error(void)
{
	read_pointer = 0;
	write_pointer = 0;
	remaining_data = 0;
	SoftwareTimer_wait_for(&timeout_timer);
	timeout_timer.state = Idle;
	error = NONE;
	state = READY;
}


static inline void send_ack(void)
{
	const uint8_t ack[4] = {ACK_PACKET_BYTE, ACK_PACKET_BYTE, ACK_PACKET_BYTE, ACK_PACKET_BYTE};
	USART_SendArray(USART3, ack, 4);
}


static inline void send_error(void)
{
	const uint8_t err[4] = {ERROR_PACKET_BYTE, ERROR_PACKET_BYTE, ERROR_PACKET_BYTE, ERROR_PACKET_BYTE};
	USART_SendArray(USART3, err, 4);
}


static inline void update_write_pointer(void)
{
	write_pointer = (write_pointer == CONTROLLER_BUF_SIZE-1) ? 0 : write_pointer+1;
}


static inline void update_read_pointer(void)
{
	read_pointer = (read_pointer == CONTROLLER_BUF_SIZE-1) ? 0 : read_pointer+1;
}


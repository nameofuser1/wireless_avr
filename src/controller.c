/*
 * controller.c
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */
#include <periph/spi.h>
#include <periph/usart3.h>
#include "controller.h"
#include "avr_flasher.h"
#include "PacketManager.h"
#include "soft_timers/SoftwareTimer2.h"

static void CONTROLLER_state_ready(void);
static void CONTROLLER_state_read_init(void);
static void CONTROLLER_state_read_cmd(void);
static void CONTROLLER_state_send_cmd(void);
static void CONTROLLER_state_terminate(void);
static void CONTROLLER_state_error(void);
static inline void send_ack(void);
static inline void send_error(void);

static ProgramState state = READY;
static ResultCode error = NONE;
static void (*actions[CONTROLLER_ACTION_NUM])(void);

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
	uint32_t available = USART3_available();
	if(available != 0 && available%4 == 0)
	{
		uint8_t buf[available];
		for(uint32_t i=0; i<available; i++)
		{
			buf[i] = USART3_read();
		}
		PacketManager_parse(buf, available);
	}

	if(PacketManager_available())
	{
		PacketType type = PacketManager_next_packet_type();

		switch(type)
		{
			case STOP_PACKET:
				PacketManager_get_packet();
				state = TERMINATE;
				break;

			case RESET_PACKET:
				PacketManager_get_packet();
				AVRFlasher_reset_disable();
				SoftwareTimer_delay_ms(&soft_timer2, 5);
				AVRFlasher_reset_enable();
				SoftwareTimer_delay_ms(&soft_timer2, 5);
				send_ack();
				break;

			default:
				break;
		}
	}

	(*actions[state])();

	return error;
}


ProgramState CONTROLLER_get_state(void)
{
	return state;
}


static void CONTROLLER_state_ready(void)
{
	if(PacketManager_available())
	{
		Packet packet = PacketManager_get_packet();
		if(packet.type == INIT_PACKET)
		{
			AVRFlasher_reset_enable();
			send_ack();

			SoftwareTimer_arm(&wait_at_ready_timer, OnePulse, 30);
			SoftwareTimer_start(&soft_timer2, &wait_at_ready_timer);
			SoftwareTimer_wait_for(&wait_at_ready_timer);

			SoftwareTimer_arm(&timeout_timer, OnePulse, ACTION_TIMEOUT_MS);
			SoftwareTimer_start(&soft_timer2, &timeout_timer);
			state = READ_CMD;
		}
		else
		{
			error = INITIAL_ERROR;
			state = FAILED;
			/* Send error??? */
		}
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
		if(PacketManager_available())
		{
			SoftwareTimer_stop(&soft_timer2, &timeout_timer);
			state = SEND_CMD;
		}
	}
}


static void CONTROLLER_state_send_cmd(void)
{
	if(PacketManager_available())
	{
		Packet cmd_packet = PacketManager_get_packet();
		AvrCommand command;
		command.b1 = cmd_packet.data[0];
		command.b2 = cmd_packet.data[1];
		command.b3  = cmd_packet.data[2];
		command.b4 = cmd_packet.data[3];

		uint8_t res[4];
		AVRFlasher_send_command(&command, res);

		USART_SendArray(USART3, res, 4);
		state = READ_CMD;

		printf("Get avr answer: ");
		for(int i=0; i<4; i++) printf("0x%02x ", res[i]);
		printf("\r\n\r\n");
	}
}


static void CONTROLLER_state_terminate(void)
{
	AVRFlasher_reset_disable();
	send_ack();
	SoftwareTimer_wait_for(&timeout_timer);
	state = READY;
}


static void CONTROLLER_state_error(void)
{
	AVRFlasher_reset_disable();
	send_error();
	SoftwareTimer_wait_for(&timeout_timer);
	error = NONE;
	state = READY;
}


void CONTROLLER_clear_error(void)
{
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



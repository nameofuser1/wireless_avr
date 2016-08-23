/*
 * controller.c
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */
#include "esp8266.h"
#include "controller.h"
#include "avr_flasher.h"
#include "PacketManager.h"
#include "soft_timers/SoftwareTimer2.h"

#include <stdio.h>
#include <inttypes.h>
#include <stm32f10x_crc.h>

static void CONTROLLER_state_ready(void);
static void CONTROLLER_state_read_prog_init(void);
static void CONTROLLER_state_read_cmd(void);
static void CONTROLLER_state_send_cmd(void);
static void CONTROLLER_state_prog_mem(void);
static void CONTROLLER_state_read_mem(void);
static void CONTROLLER_state_terminate(void);
static void CONTROLLER_state_failed(void);

static	ProgrammerType CONTROLLER_get_prog_type(uint8_t byte);
static 	ProgrammerType programmer_type = PROG_NONE;

static ProgramState state = READY;
static ResultCode error = NONE;
static void (*actions[CONTROLLER_ACTION_NUM])(void);


void CONTROLLER_init(void)
{
	printf("Controller init\r\n");
	PacketManager_init();

	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//1 ms
	SoftwareTimer2_start();

	ESP8266_init();
	ESP8266_FlushRx();
	ESP8266_flush_tx();

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_PROG_INIT] = CONTROLLER_state_read_prog_init;
	actions[READ_CMD] = CONTROLLER_state_read_cmd;
	actions[SEND_CMD] = CONTROLLER_state_send_cmd;
	actions[PROG_MEM] = CONTROLLER_state_prog_mem;
	actions[READ_MEM] = CONTROLLER_state_read_mem;
	actions[TERMINATE] = CONTROLLER_state_terminate;
	actions[FAILED] = CONTROLLER_state_failed;

	state = READY;
}


void CONTROLLER_DeInit(void)
{
	CONTROLLER_state_terminate();
}


ResultCode CONTROLLER_perform_action(void)
{
	PacketManager_parse();

	if(PacketManager_available())
	{
		PacketType type = PacketManager_next_packet_type();

		switch(type)
		{
			case STOP_PROGRAMMER_PACKET:

				ESP8266_SendAck();

				Packet packet = PacketManager_get_packet();
				PacketManager_free(packet);

				AVRFlasher_stop();
				AVRFlasher_reset_disable();

				state = READY;
				break;

			case RESET_PACKET:

				ESP8266_SendAck();

				Packet reset_packet = PacketManager_get_packet();

				if(reset_packet.data[0] == 1)
				{
					AVRFlasher_reset_enable();
				}
				else
				{
					AVRFlasher_reset_disable();
				}

				PacketManager_free(reset_packet);
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

/*
 * **************************************************
 * Program is in this state when doing nothing
 * **************************************************
 */
static void CONTROLLER_state_ready(void)
{
	if(PacketManager_available())
	{
		//printf("State ready. Packet manager available\r\n");
		Packet packet = PacketManager_get_packet();
		if(packet.type == PROG_INIT_PACKET)
		{
			//printf("State ready. Got prog init packet\r\n");
			if(!ESP8266_SendAck())
			{
				printf("Failed to send ack\r\n");
			}

			programmer_type = CONTROLLER_get_prog_type(packet.data[0]);
			state = READ_PROG_INIT;
		}
		else
		{
			//printf("State ready. Error\r\n");
			error = INITIAL_ERROR;
			state = FAILED;
		}

		PacketManager_free(packet);
	}
}


/*
 * *************************************************
 * After got PROG_INIT we should initialize
 * corresponding programmer
 * *************************************************
 */
static void CONTROLLER_state_read_prog_init(void)
{
	if(PacketManager_available())
	{
		//printf("State read prog init. PacketManager available.\r\n");
		Packet mcu_packet = PacketManager_get_packet();

		if(programmer_type == PROG_AVR)
		{
			printf("Getting mcu info in state read prog init\r\n");
			AvrMcuData mcu_info = AVRFlasher_get_mcu_info(mcu_packet);
			AVRFlasher_init(mcu_info);

			Packet en_packet = AVRFlasher_pgm_enable();
			ESP8266_SendPacket(en_packet);

			PacketManager_free(en_packet);

			state = READ_CMD;
		}
		else
		{
			error = PROG_TYPE_ERROR;
			state = FAILED;
		}

		PacketManager_free(mcu_packet);
	}
}

/*
 * **************************************************
 * Reads command packet and jumps into corresponding
 * state
 * **************************************************
 */
static void CONTROLLER_state_read_cmd(void)
{
	if(PacketManager_available())
	{
		if(PacketManager_next_packet_type() == PROG_MEM_PACKET)
		{
			state = PROG_MEM;
		}
		else if(PacketManager_next_packet_type() == CMD_PACKET)
		{
			state = SEND_CMD;
		}
		else if(PacketManager_next_packet_type() == READ_MEM_PACKET)
		{
			state = READ_MEM;
		}
		else
		{
			state = FAILED;
		}
	}
}


/*
 * ****************************************
 * Loads command into programmer
 * ****************************************
 */
static void CONTROLLER_state_send_cmd(void)
{

	Packet cmd_packet = PacketManager_get_packet();

	if(programmer_type == PROG_AVR)
	{
		uint8_t res[AVR_CMD_SIZE];
		if(AVRFlasher_send_command(cmd_packet.data, cmd_packet.data_length, res))
		{
			Packet result_packet = PacketManager_create_packet(res, AVR_CMD_SIZE, CMD_PACKET);
			ESP8266_SendPacket(result_packet);

			PacketManager_free(result_packet);
		}
		else
		{
			state = FAILED;
		}
	}

	if(state != FAILED)
	{
		state = READ_CMD;
	}

	PacketManager_free(cmd_packet);
}


/*
 * *******************************************************
 * Perfom memory programming operation
 * *******************************************************
 */
static void CONTROLLER_state_prog_mem(void)
{
	Packet memory_packet = PacketManager_get_packet();

	if(programmer_type == PROG_AVR)
	{
		AvrProgMemData mem_data = AVRFlasher_get_prog_mem_data(memory_packet);
		if(AVRFlasher_prog_memory(mem_data))
		{
			ESP8266_SendAck();
		}
		else
		{
			state = FAILED;
		}

		free(mem_data.data);
	}

	PacketManager_free(memory_packet);

	if(state != FAILED)
	{
		state = READ_CMD;
	}
}


/*
 * *******************************************
 * 	Reads memory and sends result back
 * *******************************************
 */
static void CONTROLLER_state_read_mem(void)
{
	Packet read_mem_params = PacketManager_get_packet();

	if(programmer_type == PROG_AVR)
	{
		AvrReadMemData mem_data = AVRFlasher_get_read_mem_data(read_mem_params);
		Packet memory_packet = AVRFlasher_read_mem(mem_data);

		if(!ESP8266_SendPacket(memory_packet))
		{
			printf("Failed to send memory packet. Usart overflow\r\n");
		}

		PacketManager_free(memory_packet);
		state = READ_CMD;
	}

	PacketManager_free(read_mem_params);
}


/*
 * ***********************************************
 * Stop working
 * Maybe go in sleep in future
 * ***********************************************
 */
static void CONTROLLER_state_terminate(void)
{
	ESP8266_DeInit();
	state = READY;
}


/*
 * *********************************************
 * Error routine
 * *********************************************
 */
static void CONTROLLER_state_failed(void)
{
	error = NONE;
	state = READ_CMD;
}


ProgrammerType CONTROLLER_get_prog_type(uint8_t byte)
{
	switch(byte)
	{
		case AVR_PROGRAMMER_BYTE:
			return PROG_AVR;

		case ARDUINO_PROGRAMMER_BYTE:
			return PROG_ARDUINO;

		case STM_PROGRAMMER_BYTE:
			return PROG_STM;

		default:
			return PROG_NONE;
	}
}



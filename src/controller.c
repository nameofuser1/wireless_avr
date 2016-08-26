/*
 * controller.c
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */

#include "controller.h"
#include "system.h"
#include "protocol.h"
#include "esp8266.h"
#include "avr_flasher.h"
#include "EspUpdater.h"
#include "soft_timers/SoftwareTimer2.h"
#include "common/logging.h"

#include <stdio.h>
#include <inttypes.h>
#include <stm32f10x_crc.h>


/* Controller states */
static void CONTROLLER_state_ready(void);
static void CONTROLLER_state_read_prog_init(void);
static void CONTROLLER_state_read_cmd(void);
static void CONTROLLER_state_terminate(void);
static void CONTROLLER_state_failed(void);

/* static methods */
static void load_cmd(Packet cmd_packet);
static void prog_mem(Packet mem_packet);
static void read_mem(Packet mem_info);
static void _send_ack(void);

static	ProgrammerType CONTROLLER_get_prog_type(uint8_t byte);
static 	ProgrammerType programmer_type = PROG_NONE;

static ProgramState state = READY;
static ResultCode error = NONE;
static void (*actions[CONTROLLER_ACTION_NUM])(void);


static Packet current_packet = NULL;


void CONTROLLER_init(void)
{
	/* Initialize it first because it's
	 * also responsible for printf  */
	EspUpdater_Init(115200);
	LOGGING_SetLevel(LOG_INFO);
	LOGGING_Info("Controller init\r\n");

	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//1 ms
	SoftwareTimer2_start();

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_MCU_INFO] = CONTROLLER_state_read_prog_init;
	actions[READ_CMD] = CONTROLLER_state_read_cmd;
	actions[TERMINATE] = CONTROLLER_state_terminate;
	actions[FAILED] = CONTROLLER_state_failed;

	ESP8266_Init();

	state = READY;
}


void CONTROLLER_DeInit(void)
{
	CONTROLLER_state_terminate();
}


ResultCode CONTROLLER_perform_action(void)
{
	if(ESP8266_Available())
	{
		current_packet = ESP8266_GetPacket();

		/*
		 * We do not receive Error packets
		 * Error Packet means that we have parser error
		 */
		if(current_packet->type == ERROR_PACKET)
		{

		}
	}

	(*actions[state])();

	if(current_packet != NULL)
	{
		PacketManager_free(current_packet);
		current_packet = NULL;
	}

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
	if(current_packet != NULL)
	{
		switch(current_packet->type)
		{
			case AVR_PROG_INIT_PACKET:
				_send_ack();
				programmer_type = CONTROLLER_get_prog_type(current_packet->data[0]);
				state = READ_MCU_INFO;
				break;

			case STOP_PROGRAMMER_PACKET:
				_send_ack();
				AVRFlasher_stop();
				AVRFlasher_reset_disable();
				state = READY;
				break;

			case RESET_PACKET:
				_send_ack();

				if(current_packet->data[0] == RESET_ENABLE)
				{
					AVRFlasher_reset_enable();
				}
				else
				{
					AVRFlasher_reset_disable();
				}

				break;

			default:
				error = INITIAL_ERROR;
				state = FAILED;
		}
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
	if(current_packet != NULL)
	{
		if(current_packet->type == LOAD_MCU_INFO_PACKET)
		{
			if(programmer_type == PROG_AVR)
			{
				LOGGING_Info("Getting mcu info in state read prog init\r\n");
				AvrMcuData mcu_info = AVRFlasher_get_mcu_info(current_packet);
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
		}
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
	if(current_packet != NULL)
	{
		if(current_packet->type == PROG_MEM_PACKET)
		{
			prog_mem(current_packet);
		}
		else if(current_packet->type == CMD_PACKET)
		{
			load_cmd(current_packet);
		}
		else if(current_packet->type == READ_MEM_PACKET)
		{
			read_mem(current_packet);
		}
		else
		{
			error = PACKET_TYPES_ERROR;
			state = FAILED;
		}
	}
}


/*
 * ****************************************
 * Loads command into programmer
 * ****************************************
 */
static void load_cmd(Packet cmd_packet)
{
	if(programmer_type == PROG_AVR)
	{
		uint8_t res[AVR_CMD_SIZE];
		if(AVRFlasher_send_command(cmd_packet->data, cmd_packet->data_length, res))
		{
			Packet result_packet = PacketManager_CreatePacket(res, AVR_CMD_SIZE, CMD_PACKET);
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
}


/*
 * *******************************************************
 * Perform memory programming operation
 * *******************************************************
 */
static void prog_mem(Packet mem_packet)
{
	if(programmer_type == PROG_AVR)
	{
		AvrProgMemData mem_data = AVRFlasher_get_prog_mem_data(mem_packet);
		if(AVRFlasher_prog_memory(mem_data))
		{
			ESP8266_SendAck();
		}
		else
		{
			error = IO_ERROR;
			state = FAILED;
		}

		free(mem_data.data);
	}

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
static void read_mem(Packet mem_info)
{
	if(programmer_type == PROG_AVR)
	{
		AvrReadMemData mem_data = AVRFlasher_get_read_mem_data(mem_info);
		Packet memory_packet = AVRFlasher_read_mem(mem_data);

		if(!ESP8266_SendPacket(memory_packet))
		{
			error = IO_ERROR;
			state = FAILED;
		}
		else
		{
			state = READ_CMD;
		}

		PacketManager_free(memory_packet);
	}
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

	if(error == INITIAL_ERROR)
	{
		system_error("Error while initializing device");
	}
	else if(error == PROG_TYPE_ERROR)
	{
		system_error("Unsupported programmer type");
	}
	else if(error == IO_ERROR)
	{
		io_error("Can't send command to avr");
	}
	else if(error == PACKET_TYPES_ERROR)
	{
		system_error("Wrong packet type");
	}
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


static void _send_ack(void)
{
	if(!ESP8266_SendAck())
	{
		io_error("Failed to send ack\r\n");
	}
}

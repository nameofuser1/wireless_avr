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

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_PROG_INIT] = CONTROLLER_state_read_prog_init;
	actions[READ_CMD] = CONTROLLER_state_read_cmd;
	actions[SEND_CMD] = CONTROLLER_state_send_cmd;
	actions[PROG_MEM] = CONTROLLER_state_prog_mem;
	actions[READ_MEM] = CONTROLLER_state_read_mem;
	actions[TERMINATE] = CONTROLLER_state_terminate;
	actions[FAILED] = CONTROLLER_state_failed;
}

ResultCode CONTROLLER_perform_action(void)
{
	//printf("Call packet manager parse()\r\n");
	PacketManager_parse();
	//printf("Packet manager end parsing\r\n");

	if(PacketManager_available())
	{
		printf("Packet manager available\r\n");
		PacketType type = PacketManager_next_packet_type();

		switch(type)
		{
			case STOP_PACKET:
				printf("Got stop packet\r\n");
				PacketManager_get_packet();
				state = TERMINATE;
				break;

			case RESET_PACKET:
				if(true){};

				Packet reset_packet = reset_packet = PacketManager_get_packet();
				if(reset_packet.data[0] == 1)
				{
					AVRFlasher_reset_enable();
				}
				else
				{
					AVRFlasher_reset_disable();
				}

				ESP8266_SendAck();
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
		printf("State ready. Packet manager available\r\n");
		Packet packet = PacketManager_get_packet();
		if(packet.type == PROG_INIT_PACKET)
		{
			printf("State ready. Got prog init packet\r\n");
			if(!ESP8266_SendAck())
			{
				printf("Failed to send ack\r\n");
			}

			programmer_type = CONTROLLER_get_prog_type(packet.data[0]);
			state = READ_PROG_INIT;
		}
		else
		{
			printf("State ready. Error\r\n");
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
		printf("State read prog init. PacketManager available.\r\n");
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
			printf("Got memory programming command\r\n");
			state = PROG_MEM;
		}
		else if(PacketManager_next_packet_type() == CMD_PACKET)
		{
			printf("Got command packet\r\n");
			state = SEND_CMD;
		}
		else if(PacketManager_next_packet_type() == READ_MEM_PACKET)
		{
			printf("Got read memory packet\r\n");
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
	//printf("Get avr answer: ");
	//for(int i=0; i<4; i++) printf("0x%02x ", res[i]);
	//printf("\r\n\r\n");
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
		ESP8266_SendPacket(memory_packet);

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
	AVRFlasher_stop();
	AVRFlasher_reset_disable();
	ESP8266_SendAck();
	SoftwareTimer_wait_for(&timeout_timer);
	state = READY;
}


/*
 * *********************************************
 * Error routine
 * *********************************************
 */
static void CONTROLLER_state_failed(void)
{
	//AVRFlasher_stop();
	//AVRFlasher_reset_disable();
	ESP8266_SendError(error);
	SoftwareTimer_wait_for(&timeout_timer);
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



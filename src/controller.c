/*
 * controller.c
 *
 *  Created on: 22 дек. 2015 г.
 *      Author: kripton
 */
#include <stdlib.h>
#include <string.h>

#include "system/err.h"
#include "system/system.h"
#include "protocol.h"
#include "mcu.h"

#include "esp8266.h"
#include "avr_flasher.h"
#include "UsartBridge.h"
#include "common/logging.h"

#include "controller.h"


/* Controller states */
static void CONTROLLER_state_ready(void);
static void CONTROLLER_state_read_mcu_info(void);
static void CONTROLLER_state_read_cmd(void);
static void CONTROLLER_state_terminate(void);
static void CONTROLLER_state_failed(void);

/* static methods */
static void load_cmd(Packet cmd_packet);
static void prog_mem(Packet mem_packet);
static void read_mem(Packet mem_info);
static void stop_programmer(void);
static void _send_ack(void);
static void _send_packet(Packet p);
static void _log_packet(Packet log_p);

/* Arduino or ISP */
static	ProgrammerType _get_prog_type(uint8_t byte);
static 	ProgrammerType programmer_type = PROG_NONE;

/* Tracking state */
static ProgramState state = READY;
static void (*actions[CONTROLLER_ACTION_NUM])(void);

static Packet current_packet = NULL;

/* Import global error flag */
extern uint32_t device_err;

/* From packet manager for logging */
extern char *packet_names[NONE_PACKET];


void CONTROLLER_init(void)
{
	LOGGING_Info("Initializing controller");
	MCU_Init();
	MCU_RESET_OFF();

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_MCU_INFO] = CONTROLLER_state_read_mcu_info;
	actions[READ_CMD] = CONTROLLER_state_read_cmd;
	actions[TERMINATE] = CONTROLLER_state_terminate;
	actions[FAILED] = CONTROLLER_state_failed;

	state = READY;
}


void CONTROLLER_DeInit(void)
{
	UsartBridge_Stop();
	UsartBridge_DeInit();

	MCU_RESET_OFF();
	MCU_DeInit();
}


uint32_t CONTROLLER_perform_action(void)
{
	if(ESP8266_Available())
	{
		current_packet = ESP8266_GetPacket();
		LOGGING_Info("Controller got ESP packet");

		if(current_packet == NULL)
		{
			LOGGING_Info("Current packet is null");
			state = FAILED;
		}
		else
		{
			LOGGING_Info("Got %s packet", packet_names[current_packet->type]);

			if(current_packet->type == LOG_PACKET)
			{
				_log_packet(current_packet);
			}
		}
	}

	(*actions[state])();

	if(current_packet != NULL)
	{
		PacketManager_free(current_packet);
		current_packet = NULL;
	}

	return device_err;
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

				/* Stop UsartBridge before programming */
				UsartBridge_Stop();
				UsartBridge_DeInit();

				programmer_type = _get_prog_type(current_packet->data[0]);
				state = READ_MCU_INFO;
				break;

			case USART_INIT_PACKET:
				_send_ack();
				UsartBridge_Init(current_packet);
				UsartBridge_Start();
				break;

			case USART_PACKET:
				_send_ack();
				UsartBridge_Send(current_packet);
				break;

			case RESET_PACKET:
				_send_ack();

				if(current_packet->data[0] == RESET_ENABLE)
				{
					MCU_RESET_ON();
				}
				else
				{
					MCU_RESET_OFF();
				}

				break;

			default:
				device_err = DEVICE_TYPES_ERROR;
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
static void CONTROLLER_state_read_mcu_info(void)
{
	if(current_packet != NULL)
	{
		if(current_packet->type == LOAD_MCU_INFO_PACKET)
		{
			if(programmer_type == PROG_AVR)
			{
				LOGGING_Info("Getting mcu info in state read prog init\r\n");

				AvrMcuData mcu_info = AVRFlasher_get_mcu_info(current_packet->data);
				uint8_t packet_data[1];

				AVRFlasher_Init(mcu_info);
				BOOL success = AVRFlasher_pgm_enable();
				packet_data[0] = (uint8_t)success;

				Packet en_packet = PacketManager_CreatePacket(packet_data, 1, CMD_PACKET, TRUE);
				ESP8266_SendPacket(en_packet);

				PacketManager_free(en_packet);
				state = READ_CMD;
			}
			else
			{
				device_err = DEVICE_PROGRAMMER_TYPE_ERROR;
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
		else if(current_packet->type == STOP_PROGRAMMER_PACKET)
		{
			stop_programmer();
		}
		else
		{
			device_err = DEVICE_TYPES_ERROR;
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
		if(AVRFlasher_send_command(cmd_packet->data, res))
		{
			Packet result_packet = PacketManager_CreatePacket(res, AVR_CMD_SIZE, CMD_PACKET, TRUE);
			ESP8266_SendPacket(result_packet);
			PacketManager_free(result_packet);

			state = READ_CMD;
		}
		else
		{
			device_err = DEVICE_PROGRAMMING_ERROR;
			state = FAILED;
		}
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
		AvrProgMemData mem_data = AVRFlasher_get_prog_mem_data(mem_packet->data,
				mem_packet->data_length);

		if(AVRFlasher_prog_memory(mem_data))
		{
			_send_ack();
			free(mem_data.data);
			state = READ_CMD;
		}
		else
		{
			device_err = DEVICE_PROGRAMMING_ERROR;
			state = FAILED;
		}
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
		AvrReadMemData mem_data = AVRFlasher_get_read_mem_data(mem_info->data);
		Packet memory_packet = PacketManager_CreatePacket(NULL, mem_data.bytes_to_read,
				MEMORY_PACKET, FALSE);

		AVRFlasher_read_mem(&mem_data, memory_packet->data);
		LOGGING_Debug("Read memory");
		_send_packet(memory_packet);
		LOGGING_Debug("Sending packet");
		PacketManager_free(memory_packet);
		LOGGING_Debug("Clear memory packet");
		state = READ_CMD;
	}
}


static void stop_programmer(void)
{

	_send_ack();

	AVRFlasher_DeInit();
	AVRFlasher_reset_disable();
	state = READY;
}

/*
 * ***********************************************
 * Stop working
 * Maybe go in sleep in future
 * ***********************************************
 */
static void CONTROLLER_state_terminate(void)
{
	CONTROLLER_DeInit();
	state = READY;
}


/*
 * *********************************************
 * Error routine
 * *********************************************
 */
static void CONTROLLER_state_failed(void)
{
	UsartBridge_Stop();
	UsartBridge_DeInit();

	/* Waiting for system_irq */
	while(1);
}


ProgrammerType _get_prog_type(uint8_t byte)
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


static void _send_packet(Packet p)
{
	if(!ESP8266_SendPacket(p))
	{
		io_error("Failed to send packet");
	}
}


static void _log_packet(Packet log_p)
{
	char str[log_p->data_length+1];
	memcpy(str, log_p->data, log_p->data_length);
	str[log_p->data_length+1] = '\0';
	LOGGING_Debug("ESP LOG: %s", str);
}




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
#include "UsartBridge.h"
#include "soft_timers/SoftwareTimer2.h"
#include "common/logging.h"

#include "err.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stm32f10x_crc.h>


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
static void _send_ack(void);
static void _send_packet(Packet p);
static void _log_packet(Packet log_p);
static void handle_error(uint32_t error_byte);

/* Arduino or ISP */
static	ProgrammerType _get_prog_type(uint8_t byte);
static 	ProgrammerType programmer_type = PROG_NONE;

/* Tracking state */
static ProgramState state = READY;
static void (*actions[CONTROLLER_ACTION_NUM])(void);

static Packet current_packet = NULL;

/* Initialize global error flag */
uint32_t device_err = DEVICE_OK;

/* From packet manager for logging */
extern char *packet_names[NONE_PACKET];


void CONTROLLER_init(void)
{
	/* Initialize it first because it's
	 * also responsible for printf  */
	EspUpdater_Init(115200);
	LOGGING_SetLevel(LOG_INFO);
	LOGGING_Info("Controller init");

	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//1 ms
	SoftwareTimer2_start();

	actions[READY] = CONTROLLER_state_ready;
	actions[READ_MCU_INFO] = CONTROLLER_state_read_mcu_info;
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


uint32_t CONTROLLER_perform_action(void)
{
	if(device_err != DEVICE_OK)
	{
		state = FAILED;
	}

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

			case STOP_PROGRAMMER_PACKET:
				_send_ack();
				AVRFlasher_stop();
				AVRFlasher_reset_disable();
				state = READY;
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
					AVRFlasher_reset_enable();
				}
				else
				{
					AVRFlasher_reset_disable();
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
				AvrMcuData mcu_info = AVRFlasher_get_mcu_info(current_packet);
				AVRFlasher_init(mcu_info);

				Packet en_packet = AVRFlasher_pgm_enable();
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
		if(AVRFlasher_send_command(cmd_packet->data, cmd_packet->data_length, res))
		{
			Packet result_packet = PacketManager_CreatePacket(res, AVR_CMD_SIZE, CMD_PACKET);
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
		AvrProgMemData mem_data = AVRFlasher_get_prog_mem_data(mem_packet);
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
		AvrReadMemData mem_data = AVRFlasher_get_read_mem_data(mem_info);
		Packet memory_packet = AVRFlasher_read_mem(mem_data);
		_send_packet(memory_packet);
		PacketManager_free(memory_packet);

		state = READ_CMD;
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
	handle_error(device_err);
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


static void handle_error(uint32_t error_byte)
{
	switch(error_byte)
	{
		case DEVICE_TYPES_ERROR:
			system_error("Unknown packet");
			break;

		case DEVICE_CRC_ERROR:
			system_error("Wrong crc");
			break;

		case DEVICE_HEADER_IDLE_LINE_ERROR:
			io_error("Idle line on ESP rx line while receiving headers");
			break;

		case DEVICE_BODY_IDLE_LINE_ERROR:
			io_error("Idle line on ESP rx line while receiving body");
			break;

		case DEVICE_RECEIVE_BUSY_ERROR:
			io_error("ESP Receive busy");
			break;

		case DEVICE_RECEIVE_PARAMETER_ERROR:
			io_error("ESP Receive parameter error");
			break;

		case DEVICE_SEND_BUSY_ERROR:
			io_error("ESP Send busy error");
			break;

		case DEVICE_SEND_PARAMETER_ERROR:
			io_error("ESP send parameter error");
			break;

		case DEVICE_UNKNOWN_DRIVER_ERROR:
			io_error("ESP unknown driver error");
			break;

		case DEVICE_MEMORY_ERROR:
			memory_error("ESP memory error");
			break;

		case DEVICE_LENGTH_ERROR:
			system_error("Packet length error");
			break;

		case DEVICE_PROGRAMMER_TYPE_ERROR:
			system_error("Unsupported programmer type");
			break;

		case DEVICE_INITIALIZATION_ERROR:
			system_error("Error while initializing device");
			break;

		case DEVICE_PROGRAMMING_ERROR:
			system_error("Programming error");
			break;

		default:
			system_error("Unknown error code");
	}
}

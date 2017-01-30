/*
 * avr_flasher.c
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_flasher.h"
#include "system/system.h"
#include "soft_timers/HardwareTimer.h"
#include "soft_timers/SoftwareTimer.h"
#include "periph/spi.h"
#include "common/logging.h"


#define FLASH_MEMORY_BYTE 	0x00
#define EEPROM_MEMORY_BYTE	0x01

/* Entering PGM mode parameters */
#define PGM_ENABLE_RETRIES		30
#define PGM_ENABLE_DELAY_MS		5
#define DELAY_AFTER_RESET_MS	20

/* Function for manipulating SCK when SPI is on */
static void sck_soft(void);
static void sck_hard(void);

/* Connect to ISP */
static void connect(void);

/* Some parsing functions */
static void AVRFlasher_create_memory_cmd(char *pattern, uint8_t pattern_len,
		uint32_t addr, uint8_t input, uint8_t *cmd);

static AvrMemoryType AVRFlasher_get_memory_type(uint8_t byte);

/* Contains info about programming MCU */
static AvrMcuData mcu_info;

/* Check if we loaded MCU info */
static bool initialized = false;



void AVRFlasher_Init(AvrMcuData data)
{
	mcu_info = data;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1_init();

	initialized = true;
}

/*
 * *************************************
 * Clears memory when not needed
 * *************************************
 */
void AVRFlasher_DeInit(void)
{
	if(initialized)
	{
		free(mcu_info.eeprom_read_pattern);
		free(mcu_info.eeprom_write_pattern);
		free(mcu_info.flash_load_hi_pattern);
		free(mcu_info.flash_load_lo_pattern);
		free(mcu_info.flash_read_hi_pattern);
		free(mcu_info.flash_read_lo_pattern);

		SPI1_disable();
		initialized = false;
	}
}


/*
 * *******************************************
 * Extract useful data from packet
 * Don't forget to free packet->data after all
 * *******************************************
 */
AvrMcuData AVRFlasher_get_mcu_info(Packet packet)
{
	LOGGING_Info("Getting mcu info");
	AvrMcuData init_packet;
	uint32_t k = 0;

	/*
	uint8_t m = 0;
	for(uint16_t i = 0; i<packet->data_length;i++)
	{
		printf("0x%02x\t", packet->data[i]);

		if(++m == 9)
		{
			printf("\r\n");
			m = 0;
		}

	}
	printf("\r\n");
	*/

	init_packet.flash_load_lo_len = packet->data[k++];
	init_packet.flash_load_lo_pattern = (char*)malloc(sizeof(char)*init_packet.flash_load_lo_len);

	//printf("Load lo len: %" PRIu8 "\r\nLoad lo pattern:\r\n ", init_packet.flash_load_lo_len);
	for(uint32_t i=0; i<init_packet.flash_load_lo_len; i++)
	{
		init_packet.flash_load_lo_pattern[i] = packet->data[k++];
		/*printf("0x%02x ", init_packet.flash_load_lo_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}*/
	}
	//printf("\r\n");

	init_packet.flash_load_hi_len = packet->data[k++];
	init_packet.flash_load_hi_pattern = (char*) malloc(sizeof(char)*init_packet.flash_load_hi_len);

	//printf("\r\nLoad hi len: %" PRIu8 "\r\nLoad hi pattern:\r\n ", init_packet.flash_load_hi_len);
	for(uint32_t i=0; i<init_packet.flash_load_hi_len; i++)
	{
		init_packet.flash_load_hi_pattern[i] = (char)packet->data[k++];
		/*printf("0x%02x ", init_packet.flash_load_hi_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}*/
	}
	//printf("\r\n");

	init_packet.flash_read_lo_len = packet->data[k++];
	init_packet.flash_read_lo_pattern = (char*)malloc(sizeof(char)*init_packet.flash_read_lo_len);

	//printf("\r\nRead lo len: %" PRIu8 "\r\nRead lo pattern:\r\n", init_packet.flash_read_lo_len);
	for(uint32_t i=0; i<init_packet.flash_read_lo_len; i++)
	{
		init_packet.flash_read_lo_pattern[i] = (char)packet->data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.flash_read_lo_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}
	//printf("\r\n");


	init_packet.flash_read_hi_len = packet->data[k++];
	init_packet.flash_read_hi_pattern = (char*)malloc(sizeof(char)*init_packet.flash_read_hi_len);

	//printf("\r\nRead hi len: %" PRIu8 "\r\nRead hi pattern:\r\n", init_packet.flash_read_hi_len);
	for(uint32_t i=0; i<init_packet.flash_read_hi_len; i++)
	{
		init_packet.flash_read_hi_pattern[i] = packet->data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.flash_read_hi_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.flash_wait_ms = packet->data[k++];
	//printf("\r\nFlash wait: %" PRIu8 "\r\n", init_packet.flash_wait_ms);

	init_packet.eeprom_write_len = packet->data[k++];
	init_packet.eeprom_write_pattern = (char*) malloc(sizeof(char)*init_packet.eeprom_write_len);

	//printf("\r\nEeprom write len: %" PRIu8 "\r\nEeprom write pattern:\r\n", init_packet.eeprom_write_len);
	for(uint32_t i=0; i<init_packet.eeprom_write_len; i++)
	{
		init_packet.eeprom_write_pattern[i] = packet->data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.eeprom_write_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.eeprom_read_len = packet->data[k++];
	init_packet.eeprom_read_pattern = (char*) malloc(sizeof(char)*init_packet.eeprom_read_len);

	//printf("\r\n\r\nEeprom read len: %" PRIu8 "\r\nEeprom read pattern:\r\n", init_packet.eeprom_read_len);
	for (uint32_t i=0; i<init_packet.eeprom_read_len; i++)
	{
		init_packet.eeprom_read_pattern[i] = packet->data[k++];
		//printf("%c", init_packet.eeprom_read_pattern[i]);
		/*
		printf("0x%02x ", (uint8_t)init_packet.eeprom_read_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.eeprom_wait_ms = packet->data[k++];
	//printf("\r\n\r\nEeprom wait: %" PRIu8 "\r\n", init_packet.eeprom_wait_ms);

	//printf("PGM enable: ");
	for(uint32_t i=0; i<AVR_CMD_SIZE; i++)
	{
		init_packet.pgm_enable[i] = packet->data[k++];
		/*
		printf("0x%02x ", init_packet.pgm_enable[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}

	//printf("Packet length is %" PRIu16 " bytes\r\nUsed %" PRIu32 " bytes\r\n", packet->data_length, k-1);
	LOGGING_Info("Got mcu info");

	return init_packet;
}


/*
 * *************************************************
 * Extract useful information from packet
 * Don't forget to free packet->data after all
 * *************************************************
 */
AvrProgMemData	AVRFlasher_get_prog_mem_data(Packet packet)
{
	LOGGING_Info("Getting prog mem data");
	AvrProgMemData mem_data;

	mem_data.start_address = (packet->data[0] << 24) | (packet->data[1] << 16)
			| (packet->data[2] << 8) | (packet->data[3]);

	mem_data.memory_type = AVRFlasher_get_memory_type(packet->data[4]);
	mem_data.data_len = (packet->data_length-5);
	mem_data.data = (uint8_t*) malloc(sizeof(uint8_t) * mem_data.data_len);

	memcpy(mem_data.data, packet->data+5, mem_data.data_len);

	LOGGING_Info("Got prog mem data");
	return mem_data;
}


/*
 * **********************************************
 * Extract AvrReadMemData from packet
 * Don't forger to free packet->data after all
 * **********************************************
 */
AvrReadMemData	AVRFlasher_get_read_mem_data(Packet packet)
{
	AvrReadMemData mem_data;

	mem_data.start_address = (packet->data[0] << 24) | (packet->data[1] << 16) | (packet->data[2] << 8) |
			(packet->data[3]);
	mem_data.bytes_to_read = (packet->data[4] << 24) | (packet->data[5] << 16) | (packet->data[6] << 8) |
			(packet->data[7]);

	return mem_data;
}



/*
 * *******************************************
 * Send command to AVR
 * *******************************************
 */
bool AVRFlasher_send_command(uint8_t *cmd, uint8_t len, uint8_t *res)
{
	//printf("Send command: ");
	//for(int i=0; i<len; i++) printf("0x%02x ", *(((uint8_t*)cmd)+i));
	//printf("\r\n");

	if(len != AVR_CMD_SIZE)
	{
		return false;
	}

	//printf("Answer is: ");
	for(int i=0; i<len; i++)
	{
		SPI1_write(cmd[i]);

		while(!(SPI1->SR & SPI_SR_RXNE));
		res[i] = SPI1->DR;
		//printf("0x%02x ", res[i]);
	}
	//printf("\r\n");

	return true;
}

/*
 * **************************************************
 * Writes data to corresponding memory
 * **************************************************
 */
bool AVRFlasher_prog_memory(AvrProgMemData prog_data)
{
	if(prog_data.memory_type == MEMORY_FLASH)
	{
		return AVRFlasher_prog_flash_mem(prog_data);
	}
	else if(prog_data.memory_type == MEMORY_EEPROM)
	{
		return AVRFlasher_prog_eeprom_mem(prog_data);
	}

	return false;
}

/*
 * *********************************************************
 * Loads page buffer with given commands if memory is paged
 * And writes bytes to memory if not paged.
 * And detects simple errors
 *
 * Have to send write_memory_page command after filled
 * buffer with it if memory is paged.
 *
 * All pairs of high and low bytes must be filled!
 * *********************************************************
 */
bool AVRFlasher_prog_flash_mem(AvrProgMemData prog_data)
{
	LOGGING_Info("Starting program flash memory\r\n");
	if(prog_data.data_len % 2 != 0)
	{
		return false;
	}

	uint32_t address = prog_data.start_address;
	uint8_t answer[AVR_CMD_SIZE];
	uint8_t cmd[AVR_CMD_SIZE];

	bool _delay = (mcu_info.flash_wait_ms > 0);

	for(uint8_t i=0; i<prog_data.data_len; i+=2)
	{
		uint8_t data_byte = prog_data.data[i];

		if(_delay)
		{
			delay(mcu_info.flash_wait_ms);
		}

		AVRFlasher_create_memory_cmd(mcu_info.flash_load_hi_pattern, mcu_info.flash_load_hi_len,
				address, data_byte, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, answer);

		if(_delay)
		{
			delay(mcu_info.flash_wait_ms);
		}

		AVRFlasher_create_memory_cmd(mcu_info.flash_load_lo_pattern, mcu_info.flash_load_lo_len,
				address, data_byte, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, answer);

		address++;

		/*
		 * Check for error
		 * One time for two commands
		 */
		for(uint8_t j=1; j<AVR_CMD_SIZE; j++)
		{
			if(answer[j] != cmd[j-1])
			{
				LOGGING_Error("Failed when programming. Wrong bytes returned.\r\n");
				return false;
			}
		}
	}

	LOGGING_Info("Successfully wrote flash memory\r\n");

	return true;
}

/*
 * *****************************************
 * Writes bytes to eeprom memory
 * *****************************************
 */
bool AVRFlasher_prog_eeprom_mem(AvrProgMemData prog_data)
{
	uint32_t address = prog_data.start_address;
	uint8_t answer[4] = {[0 ... 3] = 0};
	uint8_t cmd[4] = {[0 ... 3] = 0};

	for(int i=0; i<prog_data.data_len; i++)
	{
		uint8_t data_byte = prog_data.data[i];

		AVRFlasher_create_memory_cmd(mcu_info.eeprom_write_pattern, mcu_info.eeprom_write_len,
				address, data_byte, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, answer);

		address++;

		for(int j=1; j<AVR_CMD_SIZE; j++)
		{
			if(answer[j] != cmd[j-1])
			{
				LOGGING_Error("Failed while programming EEPROM memory. Wrong bytes returned.");
				return false;
			}
		}

		if(mcu_info.eeprom_wait_ms > 0)
		{
			delay(mcu_info.eeprom_wait_ms);
		}
	}
	return true;
}


/*
 * ******************************************************
 * Read memory command by command and return packet
 * With read bytes
 * ******************************************************
 */
Packet AVRFlasher_read_mem(AvrReadMemData mem_data)
{
	LOGGING_Info("Begin reading memory\r\n");
	LOGGING_Info("Start address 0x%08lx\r\n", mem_data.start_address);
	LOGGING_Info("Bytes to read %" PRIu32 "\r\n", mem_data.bytes_to_read);

	uint32_t address = mem_data.start_address;

	uint8_t answer[mem_data.bytes_to_read];
	uint8_t answer_counter = 0;

	uint8_t cmd[AVR_CMD_SIZE];
	uint8_t res[AVR_CMD_SIZE];

	for(uint32_t i=0; i<mem_data.bytes_to_read; i+=2)
	{
		AVRFlasher_create_memory_cmd(mcu_info.flash_read_hi_pattern, mcu_info.flash_read_hi_len,
				address, 0, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, res);
		answer[answer_counter++] = res[AVR_CMD_SIZE-1];

		AVRFlasher_create_memory_cmd(mcu_info.flash_read_lo_pattern, mcu_info.flash_read_lo_len,
				address, 0, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, res);
		answer[answer_counter++] = res[AVR_CMD_SIZE-1];

		address++;
	}

	LOGGING_Info("Successfully read memory\r\n");
	return PacketManager_CreatePacket(answer, mem_data.bytes_to_read, MEMORY_PACKET);
}


/*
 * *********************************************
 * Trying to synchronize with controller
 *
 * *********************************************
 */
Packet AVRFlasher_pgm_enable(void)
{
	LOGGING_Info("Trying to enter into programming mode\r\n");

	uint8_t res[AVR_CMD_SIZE];
	uint8_t success[1] = {0};

	connect();

	for(uint32_t i=0; i<PGM_ENABLE_RETRIES; i++)
	{
		SPI1_enable();
		AVRFlasher_send_command(mcu_info.pgm_enable, AVR_CMD_SIZE, res);

		if(res[2] == mcu_info.pgm_enable[1])
		{
			LOGGING_Debug("Successfully entered programming mode. %" PRIu32 " retries\r\n", i+1);
			success[0] = 1;
			break;
		}
		else
		{
			LOGGING_Debug("AVR answer: %" PRIu8 " %" PRIu8 " %" PRIu8 " %" PRIu8,
					res[0], res[1], res[2], res[3]);

			SPI1_disable();

			delay(PGM_ENABLE_DELAY_MS);
			AVRFlasher_reset_disable();
			delay(PGM_ENABLE_DELAY_MS);
			AVRFlasher_reset_enable();
			delay(PGM_ENABLE_DELAY_MS);

			SPI1_enable();
		}
	}

	return PacketManager_CreatePacket(success, 1, CMD_PACKET);
}


void AVRFlasher_reset_enable(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}


void AVRFlasher_reset_disable(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}


/*
 * ********************************************************************
 * Miss comparing with 'x' and 'o' as memory filled with zeroes
 * Works for load, write and read commands
 * ********************************************************************
 */
static void
AVRFlasher_create_memory_cmd(char *pattern, uint8_t pattern_len, uint32_t addr, uint8_t input, uint8_t *cmd)
{
	//printf("Begin memory cmd creation\r\n");
	memset(cmd, 0, sizeof(uint8_t)*AVR_CMD_SIZE);

	uint8_t cmd_byte_offset = 0;
	int8_t cmd_bit_offset = 7;
	uint8_t cmd_input_offset = 7;

	for(uint8_t j=0; j<pattern_len; j++)
	{
		if(pattern[j] == '1')
		{
			cmd[cmd_byte_offset] |= (1 << cmd_bit_offset);
		}
		else if(pattern[j] == '0')
		{
			cmd[cmd_byte_offset] &= ~(0 << cmd_bit_offset);
		}
		else if(pattern[j] == 'a')
		{
			j++;
			uint8_t k = 0;
			char ch_address_shift[3];

			while((pattern[j] >= '0') && (pattern[j] <= '9'))
			{
				ch_address_shift[k++] = pattern[j++];
			}
			/* last j++ plus j++ in cycle causes missing next char */
			j--;
			ch_address_shift[k] = '\0';

			uint32_t address_shift = atoi(ch_address_shift);
			uint8_t address_bit = (addr >> address_shift) & 0x01;
			cmd[cmd_byte_offset] |= (address_bit << cmd_bit_offset);
		}
		else if(pattern[j] == 'i')
		{
			uint8_t input_bit = (input >> cmd_input_offset) & 0x01;
			cmd[cmd_byte_offset] |= (input_bit << cmd_bit_offset);
			cmd_input_offset--;
		}

		if(--cmd_bit_offset < 0)
		{
			cmd_bit_offset = 7;
			cmd_byte_offset += 1;
		}
	}

	//printf("Created command is: ");
	//for(uint32_t i=0; i<AVR_CMD_SIZE; i++)
	//{
		//printf("0x%02x ", cmd[i]);
	//}
}


static AvrMemoryType AVRFlasher_get_memory_type(uint8_t byte)
{
	switch(byte)
	{
		case FLASH_MEMORY_BYTE:
			return MEMORY_FLASH;

		case EEPROM_MEMORY_BYTE:
			return MEMORY_EEPROM;
	}

	return MEMORY_ERR;
}


static void connect(void)
{
	sck_soft();
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);	// pull sck down
	AVRFlasher_reset_enable();			// pull reset down

	delay(PGM_ENABLE_DELAY_MS);
	AVRFlasher_reset_disable();
	delay(PGM_ENABLE_DELAY_MS);
	AVRFlasher_reset_enable();

	sck_hard();
	SPI1_init();
	SPI1_enable();
}



static void sck_soft(void) {
	GPIO_InitTypeDef sck;
	sck.GPIO_Mode = GPIO_Mode_Out_PP;
	sck.GPIO_Pin = GPIO_Pin_5;
	sck.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &sck);
}


static void sck_hard(void) {
	GPIO_InitTypeDef sck;
	sck.GPIO_Mode = GPIO_Mode_AF_PP;
	sck.GPIO_Pin = GPIO_Pin_5;
	sck.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &sck);

}





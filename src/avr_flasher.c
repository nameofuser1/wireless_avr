/*
 * avr_flasher.c
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */
#include "avr_flasher.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "soft_timers/SoftwareTimer.h"
#include "soft_timers/SoftwareTimer2.h"
#include <periph/spi.h>
#include <inttypes.h>

#define FLASH_MEMORY_BYTE 	0x00
#define EEPROM_MEMORY_BYTE	0x01

#define PGM_ENABLE_RETRIES		30
#define PGM_ENABLE_DELAY_MS		4

#define DELAY_AFTER_RESET_MS	20

static void
AVRFlasher_create_memory_cmd(char *pattern, uint8_t pattern_len, uint32_t addr, uint8_t input, uint8_t *cmd);

static AvrMemoryType AVRFlasher_get_memory_type(uint8_t byte);

static AvrMcuData mcu_info;
static bool initialized = false;

/*
 * ******************************************************
 * Set necessary data
 * ******************************************************
 */
void AVRFlasher_init(AvrMcuData data)
{
	mcu_info = data;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1_init();
	SPI1_enable();

	initialized = true;

}

/*
 * *************************************
 * Clears memory when not needed
 * *************************************
 */
void AVRFlasher_stop(void)
{
	if(initialized)
	{
		free(mcu_info.eeprom_read_pattern);
		free(mcu_info.eeprom_write_pattern);
		free(mcu_info.flash_load_hi_pattern);
		free(mcu_info.flash_load_lo_pattern);
		free(mcu_info.flash_read_hi_pattern);
		free(mcu_info.flash_read_lo_pattern);
		initialized = false;
	}

}


/*
 * *******************************************
 * Extract useful data from packet
 * Don't forget to free packet.data after all
 * *******************************************
 */
AvrMcuData AVRFlasher_get_mcu_info(Packet packet)
{
	printf("Getting mcu info\r\n");
	AvrMcuData init_packet;
	uint32_t k = 0;

	/*
	uint8_t m = 0;
	for(uint16_t i = 0; i<packet.data_length;i++)
	{
		printf("0x%02x\t", packet.data[i]);

		if(++m == 9)
		{
			printf("\r\n");
			m = 0;
		}

	}
	printf("\r\n");
	*/

	init_packet.flash_load_lo_len = packet.data[k++];
	init_packet.flash_load_lo_pattern = (char*)malloc(sizeof(char)*init_packet.flash_load_lo_len);

	//printf("Load lo len: %" PRIu8 "\r\nLoad lo pattern:\r\n ", init_packet.flash_load_lo_len);
	for(uint32_t i=0; i<init_packet.flash_load_lo_len; i++)
	{
		init_packet.flash_load_lo_pattern[i] = packet.data[k++];
		/*printf("0x%02x ", init_packet.flash_load_lo_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}*/
	}
	//printf("\r\n");

	init_packet.flash_load_hi_len = packet.data[k++];
	init_packet.flash_load_hi_pattern = (char*) malloc(sizeof(char)*init_packet.flash_load_hi_len);

	//printf("\r\nLoad hi len: %" PRIu8 "\r\nLoad hi pattern:\r\n ", init_packet.flash_load_hi_len);
	for(uint32_t i=0; i<init_packet.flash_load_hi_len; i++)
	{
		init_packet.flash_load_hi_pattern[i] = (char)packet.data[k++];
		/*printf("0x%02x ", init_packet.flash_load_hi_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}*/
	}
	//printf("\r\n");

	init_packet.flash_read_lo_len = packet.data[k++];
	init_packet.flash_read_lo_pattern = (char*)malloc(sizeof(char)*init_packet.flash_read_lo_len);

	//printf("\r\nRead lo len: %" PRIu8 "\r\nRead lo pattern:\r\n", init_packet.flash_read_lo_len);
	for(uint32_t i=0; i<init_packet.flash_read_lo_len; i++)
	{
		init_packet.flash_read_lo_pattern[i] = (char)packet.data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.flash_read_lo_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}
	//printf("\r\n");


	init_packet.flash_read_hi_len = packet.data[k++];
	init_packet.flash_read_hi_pattern = (char*)malloc(sizeof(char)*init_packet.flash_read_hi_len);

	//printf("\r\nRead hi len: %" PRIu8 "\r\nRead hi pattern:\r\n", init_packet.flash_read_hi_len);
	for(uint32_t i=0; i<init_packet.flash_read_hi_len; i++)
	{
		init_packet.flash_read_hi_pattern[i] = packet.data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.flash_read_hi_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.flash_wait_ms = packet.data[k++];
	//printf("\r\nFlash wait: %" PRIu8 "\r\n", init_packet.flash_wait_ms);

	init_packet.eeprom_write_len = packet.data[k++];
	init_packet.eeprom_write_pattern = (char*) malloc(sizeof(char)*init_packet.eeprom_write_len);

	//printf("\r\nEeprom write len: %" PRIu8 "\r\nEeprom write pattern:\r\n", init_packet.eeprom_write_len);
	for(uint32_t i=0; i<init_packet.eeprom_write_len; i++)
	{
		init_packet.eeprom_write_pattern[i] = packet.data[k++];
		/*
		printf("0x%02x ", (uint8_t)init_packet.eeprom_write_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.eeprom_read_len = packet.data[k++];
	init_packet.eeprom_read_pattern = (char*) malloc(sizeof(char)*init_packet.eeprom_read_len);

	//printf("\r\n\r\nEeprom read len: %" PRIu8 "\r\nEeprom read pattern:\r\n", init_packet.eeprom_read_len);
	for (uint32_t i=0; i<init_packet.eeprom_read_len; i++)
	{
		init_packet.eeprom_read_pattern[i] = packet.data[k++];
		//printf("%c", init_packet.eeprom_read_pattern[i]);
		/*
		printf("0x%02x ", (uint8_t)init_packet.eeprom_read_pattern[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}


	init_packet.eeprom_wait_ms = packet.data[k++];
	//printf("\r\n\r\nEeprom wait: %" PRIu8 "\r\n", init_packet.eeprom_wait_ms);

	//printf("PGM enable: ");
	for(uint32_t i=0; i<AVR_CMD_SIZE; i++)
	{
		init_packet.pgm_enable[i] = packet.data[k++];
		/*
		printf("0x%02x ", init_packet.pgm_enable[i]);
		if(i%10 == 9)
		{
			printf("\r\n");
		}
		*/
	}

	//printf("Packet length is %" PRIu16 " bytes\r\nUsed %" PRIu32 " bytes\r\n", packet.data_length, k-1);
	printf("\r\nMcu info got\r\n");

	return init_packet;
}


/*
 * *************************************************
 * Extract useful information from packet
 * Don't forget to free packet.data after all
 * *************************************************
 */
AvrProgMemData	AVRFlasher_get_prog_mem_data(Packet packet)
{
	//printf("Getting prog mem data\r\n");
	AvrProgMemData mem_data;

	mem_data.start_address = (packet.data[0] << 24) | (packet.data[1] << 16) | (packet.data[2] << 8) |
			(packet.data[3]);
	mem_data.memory_type = AVRFlasher_get_memory_type(packet.data[4]);

	mem_data.data_len = (packet.data_length-5);
	mem_data.data = (uint8_t*) malloc(sizeof(uint8_t) * mem_data.data_len);

	memcpy(mem_data.data, packet.data+5, mem_data.data_len);

	//printf("Got prog mem data\r\n");
	return mem_data;
}


/*
 * **********************************************
 * Extract AvrReadMemData from packet
 * Don't forger to free packet.data after all
 * **********************************************
 */
AvrReadMemData	AVRFlasher_get_read_mem_data(Packet packet)
{
	printf("Getting read mem data\r\n");
	AvrReadMemData mem_data;

	mem_data.start_address = (packet.data[0] << 24) | (packet.data[1] << 16) | (packet.data[2] << 8) |
			(packet.data[3]);
	mem_data.bytes_to_read = (packet.data[4] << 24) | (packet.data[5] << 16) | (packet.data[6] << 8) |
			(packet.data[7]);

	printf("Got read mem data\r\n");

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
	printf("Starting program flash memory\r\n");
	if(prog_data.data_len % 2 != 0)
	{
		return false;
	}

	uint32_t address = prog_data.start_address;
	uint8_t answer[AVR_CMD_SIZE];
	uint8_t cmd[AVR_CMD_SIZE];

	bool delay = (mcu_info.flash_wait_ms > 0);

	for(uint8_t i=0; i<prog_data.data_len; i+=2)
	{
		uint8_t data_byte = prog_data.data[i];

		if(delay)
		{
			SoftwareTimer_delay_ms(&soft_timer2, mcu_info.flash_wait_ms);
		}

		AVRFlasher_create_memory_cmd(mcu_info.flash_load_hi_pattern, mcu_info.flash_load_hi_len,
				address, data_byte, cmd);
		AVRFlasher_send_command(cmd, AVR_CMD_SIZE, answer);

		if(delay)
		{
			SoftwareTimer_delay_ms(&soft_timer2, mcu_info.flash_wait_ms);
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
				printf("Failed when programming. Wrong bytes returned.\r\n");
				return false;
			}
		}
	}

	printf("Successfully wrote flash memory\r\n");

	return true;
}

/*
 * *****************************************
 * Writes bytes to eeprom memory
 * *****************************************
 */
bool AVRFlasher_prog_eeprom_mem(AvrProgMemData prog_data)
{
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
	printf("Begin reading memory\r\n");
	uint32_t address = mem_data.start_address;

	uint8_t answer[mem_data.bytes_to_read];
	uint8_t answer_counter = 0;

	uint8_t cmd[AVR_CMD_SIZE];
	uint8_t res[AVR_CMD_SIZE];

	for(uint8_t i=0; i<mem_data.bytes_to_read; i+=2)
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

	printf("Successfully read memory\r\n");
	return PacketManager_create_packet(answer, mem_data.bytes_to_read, MEMORY_PACKET);
}


/*
 * *********************************************
 * Trying to synchronize with controller
 *
 * *********************************************
 */
Packet AVRFlasher_pgm_enable(void)
{
	printf("Trying to enter into programming mode\r\n");
	uint8_t res[AVR_CMD_SIZE];
	uint8_t success[1] = {0};

	for(uint32_t i=0; i<PGM_ENABLE_RETRIES; i++)
	{
		AVRFlasher_reset_enable();
		SoftwareTimer_delay_ms(&soft_timer2, DELAY_AFTER_RESET_MS);

		AVRFlasher_send_command(mcu_info.pgm_enable, AVR_CMD_SIZE, res);

		if(res[2] == mcu_info.pgm_enable[1])
		{
			printf("Successfully entered programming mode\r\n");
			success[0] = 1;
			break;
		}
		else
		{
			AVRFlasher_reset_disable();
			SoftwareTimer_delay_ms(&soft_timer2, PGM_ENABLE_DELAY_MS);
		}
	}

	return PacketManager_create_packet(success, 1, CMD_PACKET);
}


void AVRFlasher_reset_enable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BR8;
}


void AVRFlasher_reset_disable(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BS8;
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

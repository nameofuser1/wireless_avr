/*
 * avr_flasher.h
 *
 *  Created on: 24 нояб. 2015 г.
 *      Author: kripton
 */

#ifndef AVR_FLASHER_H_
#define AVR_FLASHER_H_

#include "stm32f10x.h"
#include <stdbool.h>
#include "PacketManager.h"

#define AVR_CMD_SIZE 	4
#define AVR_WORD_SIZE	2

typedef enum { MEMORY_FLASH = 0, MEMORY_EEPROM, MEMORY_ERR} AvrMemoryType;

typedef struct {

	uint8_t cmd[4];

} AvrCommand;


typedef struct {

	uint8_t 	flash_load_hi_len;
	char 		*flash_load_hi_pattern;

	uint8_t		flash_load_lo_len;
	char		*flash_load_lo_pattern;

	uint8_t		flash_read_lo_len;
	char		*flash_read_lo_pattern;

	uint8_t		flash_read_hi_len;
	char		*flash_read_hi_pattern;

	uint8_t		flash_wait_ms;

	uint8_t		eeprom_write_len;
	char		*eeprom_write_pattern;

	uint8_t 	eeprom_read_len;
	char		*eeprom_read_pattern;

	uint8_t		eeprom_wait_ms;

	uint8_t		pgm_enable[AVR_CMD_SIZE];


} AvrMcuData;


typedef struct {

	uint32_t 		start_address;
	AvrMemoryType	memory_type;
	uint8_t 		data_len;
	uint8_t 		*data;

} AvrProgMemData;

typedef struct {

	uint32_t start_address;
	uint32_t bytes_to_read;

} AvrReadMemData;


void 			AVRFlasher_Init(AvrMcuData data);
void 			AVRFlasher_DeInit(void);

AvrMcuData		AVRFlasher_get_mcu_info(Packet packet);
AvrProgMemData	AVRFlasher_get_prog_mem_data(Packet packet);
AvrReadMemData	AVRFlasher_get_read_mem_data(Packet packet);

bool 			AVRFlasher_send_command(uint8_t *cmd, uint8_t len, uint8_t *res);
bool 			AVRFlasher_prog_memory(AvrProgMemData mem_data);
bool			AVRFlasher_prog_flash_mem(AvrProgMemData mem_data);
bool 			AVRFlasher_prog_eeprom_mem(AvrProgMemData prog_data);
Packet 			AVRFlasher_read_mem(AvrReadMemData mem_data);
Packet			AVRFlasher_pgm_enable(void);

void 			AVRFlasher_reset_enable(void);
void 			AVRFlasher_reset_disable(void);

#endif /* AVR_FLASHER_H_ */

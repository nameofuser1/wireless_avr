#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include "stm32f10x.h"
#include <stdbool.h>

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)

bool FLASH_is_locked(void);
void FLASH_unlock(void);
void FLASH_lock(void);
bool FLASH_ready(void);

uint32_t 	FLASH_read_word(uint32_t addr);
void		FLASH_read_block(uint32_t start_addr, uint32_t *buf, uint16_t len);
bool		FLASH_write_word(uint32_t addr, uint32_t data);
void 		FLASH_erase_page(uint32_t addr);
void 		FLASH_mass_erase(void);

#endif

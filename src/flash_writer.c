/*
 * flash_writer.c
 *
 *  Created on: 22 нояб. 2015 г.
 *      Author: kripton
 */
#include "flash_writer.h"


static inline void FLASH_write_half_word(uint32_t addr, uint16_t data);


/*
 * **************************************
 * Check for FPEC is locked
 * **************************************
 */
bool FLASH_is_locked(void)
{
	return (FLASH->CR & FLASH_CR_LOCK);
}

/*
 * **************************************
 * Unlock FPEC
 * **************************************
 */
void FLASH_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}

/*
 * ***************************************
 * Lock FPEC
 * ***************************************
 */
void FLASH_lock(void)
{
	FLASH->CR = FLASH_CR_LOCK;
}


/*
 * ********************************************
 * Check for previous operation is completed
 * ********************************************
 */
bool FLASH_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

/*
 * ******************************************
 * Read word (4bytes) of flash with given
 * address
 * ******************************************
 */
uint32_t FLASH_read_word(uint32_t addr)
{
	return *(__IO uint32_t*)addr;
}


/*
 * *************************************************
 * Reads pages block of length len to given buffer
 * Be careful and do not got beyond memory
 * *************************************************
 */
void FLASH_read_block(uint32_t start_addr, uint32_t *buf, uint16_t len)
{
	uint16_t i = 0;
	for(i = 0; i<len; i++)
	{
		buf[i] = FLASH_read(start_addr);
		start_addr+=4;
	}
}


/*
 * *******************************************************
 * Write data to flash memory with given address
 * Note: you should unlock writing with FLASH_unlock first
 * *******************************************************
 */
bool FLASH_write_word(uint32_t addr, uint32_t data)
{
	FLASH_erase_page(addr);
	FLASH->CR |= FLASH_CR_PG;
	FLASH_write_half_word(addr, data);
	while(!FLASH_ready());
	FLASH_write_half_word(addr+2, data);
	while(!FLASH_ready());
	FLASH->CR &= ~FLASH_CR_PG;

	if((FLASH->SR & FLASH_SR_PGERR) || (FLASH->SR & FLASH_SR_WRPRTERR))
	{
		return false;
	}

	return true;
}


/*
 * *******************************************
 * Erase one page of flash with given address
 * *******************************************
 */
void FLASH_erase_page(uint32_t addr)
{
	while(!FLASH_ready());
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = addr;
	FLASH->CR |= FLASH_CR_STRT;
	while(!FLASH_ready());
	FLASH->CR &= ~FLASH_CR_PER;
}


/*
 * ********************************************
 * Erase all the flash
 * Note: it also erases your firmware
 * ********************************************
 */
void FLASH_mass_erase(void)
{
	while(!FLASH_ready());
	FLASH->CR |= FLASH_CR_MER;
	FLASH->CR |= FLASH_CR_STRT;
	while(!FLASH_ready());
	FLASH->CR &= ~FLASH_CR_MER;
}


/*
 * *****************************************************
 * We can write to flash only half-word size data
 * *****************************************************
 */
static inline void FLASH_write_half_word(uint32_t addr, uint16_t data)
{
	*(__IO uint32_t*)addr = data;
}

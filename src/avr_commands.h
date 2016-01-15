#ifndef AVR_COMMANDS_H
#define AVR_COMMANDS_H

/* Only for types. Maybe another file? */
#include "stm32f10x.h"

/*
 * Memory addresses
 */
#define AT16_FLASH_START 	0x0000
#define AT16_FLASH_END		0x4000	//16Kbytes

/*
 * Set it as fourth byte if command returns answer
 */
#define AT16_ANSWER_BYTE		0xAA


/*
 * *******************************************************************
 *							COMMON INSTRUCTIONS						 *
 * *******************************************************************
*/

/*
 * Programming Enable Instruction
 */
#define AT16_PROG_EN_B1		0xAC
#define AT16_PROG_EN_B2		0x53
#define	AT16_PROG_EN_B3		0x00
#define AT16_PROG_EN_B4		0x00

/*
 * Mass erase instruction
 */
#define AT16_CHP_ERS_B1		0xAC
#define AT16_CHP_ERS_B2		0x80
#define	AT16_CHP_ERS_B3		0x00
#define	AT16_CHP_ERS_B4		0x00

/*
 *	Status poll instruction
 *	Fourth byte is the answer
 */
#define AT16_POLL_B1	0xF0
#define AT16_POLL_B2	0x00
#define	AT16_POLL_B3	0x00
#define AT16_POLL_B4	AT16_ANSWER_BYTE


/*
 * *******************************************************************
 *							LOAD INSTRUCTIONS						 *
 * *******************************************************************
*/


/*
 *	Load extended address instruction
 *	Third byte is extended address
 */
#define AT16_LD_EXT_ADDR_B1		0x4D
#define AT16_LD_EXT_ADDR_B2		0x00
#define AT16_LD_EXT_ADDR_B4		0x00

/*
 * Load program memory high byte
 * Second byte is addr MSB
 * Third byte is addr  LSB
 * Fourth byte is data high byte
 */
#define AT16_LD_HPG_MEM_B1		0x48

/*
 * Load program memory low byte
 * Second byte is addr MSB
 * Third byte is addr  LSB
 * Fourth byte is data low byte
 */
#define AT16_LD_LPG_MEM_B1		0x40


/*
 * Load EEPROM memory page instruction
 * Third byte is addr LSB
 * Fourth byte is data byte
 */
#define AT16_LD_EMEM_B1		0xC1
#define AT16_LD_EMEM_B2		0x00



/*
 * *******************************************************************
 *							READ INSTRUCTIONS						 *
 * *******************************************************************
*/


/*
 *	Read program memory high byte
 *	Second byte is addr MSB
 *	Third byte is addr LSB
 *	Fourth byte is high byte answer
 */
#define AT16_RD_HPG_MEM_B1	0x28
#define AT16_RD_HPG_MEM_B4	AT16_ANSWER_BYTE

/*
 *	Read program memory low byte
 *	Second byte is addr MSB
 *	Third byte is addr LSB
 *	Fourth byte is low byte answer
 */
#define AT16_RD_LPG_MEM_B1	0x20
#define AT16_RD_LPG_MEM_B4	AT16_ANSWER_BYTE

/*
 * Read EEPROM memory
 * Second byte is addr MSB
 * Third byte is addr LSB
 * Fourth byte is answer
 */
#define AT16_RD_EMEM_B1		0xA0
#define AT16_RD_EMEM_B4		AT16_ANSWER_BYTE

/*
 *	Read lock bits
 */
#define AT16_RD_LCK_B1		0x58
#define AT16_RD_LCK_B2		0x00
#define AT16_RD_LCK_B3		0x00
#define AT16_RD_LCK_B4		AT16_ANSWER_BYTE

/*
 * Read signature byte
 * LSB bits of third byte[1:0] used for address
 * Mask is used to set only last bits
 */
#define AT16_RD_SIG_B1			0x30
#define AT16_RD_SIG_B2			0x00
#define AT16_RD_SIG_B4			AT16_ANSWER_BYTE
#define AT16_RD_VENDOR_CODE		0x00
#define AT16_RD_PART_FAMILY		0x01
#define AT16_RD_PART_NUMBER		0x02

#define AT16_VENDOR_CODE		0x1E
#define AT16_PART_FAMILY
#define AT16_PART_NUMBER

/*
 * Read fuse bits (as I understand LOW fuse)
 */
#define AT16_RD_LFUSE_B1	0x50
#define AT16_RD_LFUSE_B2	0x00
#define AT16_RD_LFUSE_B3	0x00
#define AT16_RD_LFUSE_B4	ASNWER_BYTE

/*
 * Read high fuse bits
 */
#define AT16_RD_HFUSE_B1	0x58
#define AT16_RD_HFUSE_B2	0x08
#define AT16_RD_HFUSE_B3	0x00
#define AT16_RD_HFUSE_B4	ASNWER_BYTE

/*
 * Read extended fuse bits
 */
#define AT16_RD_EXFUSE_B1	0x50
#define AT16_RD_EXFUSE_B2	0x08
#define AT16_RD_EXFUSE_B3	0x00
#define AT16_RD_EXFUSE_b4	AT16_ANSWER_BYTE

/*
 *	Read calibration byte
 *	Third byte consists from magic symbols
 *	Read the datasheet
 */
#define AT16_RD_CAL_B1		0x38
#define AT16_RD_CAL_B2		0x00
#define AT16_RD_CAL_B4		AT16_ANSWER_BYTE


/*
 * *******************************************************************
 *							WRITE INSTRUCTIONS						 *
 * *******************************************************************
*/


/*
 * Write program memory page
 * Second byte LSB bits[4:0] contains addr MSB
 * Third byte MSB bits[7:6]	contains add LSB
 */
#define AT16_WRT_MEM_B1			0x4C
#define	AT16_WRT_MEM_B4			0x00
#define AT16_WRT_MEM_B2_MASK	0x1F
#define AT16_WRT_MEM_B3_MASK	0xC0


/*
 * Write EEPROM memory
 * Second byte is addr MSB
 * Third byte is addr LSB
 * Fourth byte is data to write
 */
#define AT16_WRT_EMEM_B1		0xC0


/*
 * Write memory page
 * Second byte is addr MSB
 * Third byte is addr LSB
 */
#define AT16_WRT_EMEM_PAGE_B1	0xC2
#define AT16_WRT_EMEM_PAGE_B4	0x00

/*
 * Write lock bits
 * Fourth byte is data
 */
#define AT16_WRT_LCK_B1		0xAC
#define AT16_WRT_LCK_B2		0xE0
#define AT16_WRT_LCK_B3		0x00

/*
 * Write fuse bits(as I understand LOW)
 * fourth byte is data
 */
#define AT16_WRT_LFUSE_B1	0xAC
#define AT16_WRT_LFUSE_B2	0xA0
#define AT16_WRT_LFUSE_B3	0x00

/*
 * Write high fuse bits
 * Fourth byte is data
 */
#define AT16_WRT_HFUSE_B1	0xAC
#define AT16_WRT_HFUSE_B2	0xA8
#define AT16_WRT_HFUSE_B3	0x00

/*
 *	Write extended fuse bits
 *	Fourth byte is data
 */
#define AT16_WRT_EXTFUSE_B1		0xAC
#define AT16_WRT_EXTFUSE_B2		0xA4
#define AT16_WRT_EXTFUSE_B3		0x00



typedef struct {

	uint8_t b1;
	uint8_t b2;
	uint8_t b3;
	uint8_t b4;

} AvrCommand;


#endif

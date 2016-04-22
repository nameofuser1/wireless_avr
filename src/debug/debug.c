/*
 * debug.c
 *
 *  Created on: 15 апр. 2016 г.
 *      Author: kripton
 */
#include "stm32f10x.h"

#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"


static void HardFault_Handler(void) __attribute((naked));
static void prvGetRegisterFromStack(uint32_t *pulFaultStackAddress);


static void HardFault_Handler(void)
{
	__asm volatile
	(
			"tst lr, #4													\r\n"
			"ite eq														\r\n"
			"mrseq r0, msp												\r\n"
			"mrsne r0, psp												\r\n"
			"ldr r1, [r0, #24]											\r\n"
			"ldr r2, handler2_address_const								\r\n"
			"bx r2														\r\n"
			"handler2_address_const: .word prvGetRegisterFromStack		\r\n"
	);
}


static void prvGetRegisterFromStack(uint32_t *pulFaultStackAddress)
{
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr;
	volatile uint32_t pc;
	volatile uint32_t psr;

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];
	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	for(;;);
}
















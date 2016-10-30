/*
 * system.c
 *
 *  Created on: 22 авг. 2016 г.
 *      Author: bigmac
 */
#include <stm32f10x.h>
#include <core_cm3.h>

#include "soft_timers/HardwareTimer.h"
#include "common/logging.h"
#include <stdlib.h>
#include <system/err.h>
#include <system/system.h>

#define SYSTEM_ERROR		0
#define SYSTEM_ERROR_MEM	1
#define SYSTEM_ERROR_IO		2
#define SYSTEM_ERRORS_NUM	3

static char* errors[SYSTEM_ERRORS_NUM] = {"System error", "Memory error", "IO error"};

/*
 *	One microsecond per timer tick
 *	import HardwareTimer based on TIM2
 */
#define SYSTEM_TIMER_PRESCALER 	36000-1
extern 	HardwareTimerDriver 	SystemTimer;


/*
 *	Timer for handling errors
 *	Callback for timer
 *	And error handler
 */
SoftwareTimer 	err_timer;
static void 	err_timer_cb(void);
extern void		handle_error(uint32_t error_code);


/*
 *	Import global error flag
 */
extern uint32_t device_err;


void system_init(void)
{
	SystemTimer.Init(SYSTEM_TIMER_PRESCALER);
	SystemTimer.SetDuration(1);					//update interrupt every 1 tick
	SystemTimer.Start();

	SoftwareTimer_Init(&err_timer);
	SoftwareTimer_Add_cb(&err_timer, (SoftTimerCallback)err_timer_cb);
	SoftwareTimer_Arm(&err_timer, Timer_Repeat, 1);

	SystemTimer.AddTimer(&err_timer);
}


/*
 * 	Restart controller
 */
static void critical_error(uint8_t err, char *msg)
{
	LOGGING_Error(errors[err]);

	if(msg != NULL)
	{
		LOGGING_Error(msg);
	}

	LOGGING_Error("Resetting device");
	/* Small delay for delivering LOG message */
	for(volatile uint32_t i=0; i<10000000; i++);
	NVIC_SystemReset();
}


void system_error(char *msg)
{
	critical_error(SYSTEM_ERROR, msg);
}


void memory_error(char *msg)
{
	critical_error(SYSTEM_ERROR_MEM, msg);
}


void io_error(char *msg)
{
	critical_error(SYSTEM_ERROR_IO, msg);
}

/*
 * Bug, we need to set error instead of calling error function
 * Haven't fixed it yet because I do not expect memory error
 */
void* sys_malloc(int32_t size)
{
	void *arr = malloc(size);
	if(arr == NULL)
	{
		critical_error(SYSTEM_ERROR_MEM, NULL);
	}

	return arr;
}


void delay(uint32_t ms)
{
	SoftwareTimer	delay_timer;
	SoftwareTimer_Arm(&delay_timer, Timer_OnePulse, ms);
	SystemTimer.AddTimer(&delay_timer);

	SoftwareTimer_WaitFor(&delay_timer);
}


/*
 * import SystemTimer IRQ first operation address
 * You can find it in TIM2_IRQHandler in HardwareTimer.c
 */
extern uint32_t enter_addr;


/*
 * If error is found the we change interrupt return address to
 * handler_error function address
 */
static void err_timer_cb(void)
{
	if(device_err != DEVICE_OK)
	{
		SystemTimer.Stop();
		asm(
			"ldr r5, =enter_addr\n\t"
			"ldr r5, [r5]\n\t"							// r5 points to lr (0x08 after interrupt SP)

			"add r6, r5, #0x08\n\t"						// r6 points to stacked r0

			"ldr r7, =device_err\n\t"
			"ldr r7, [r7]\n\t"							// load error code into r7
			"str r7, [r6]\n\t"							// store error code

			"add r6, r6, #0x18\n\t"						// r6 points to PC address
			"ldr r7, =(handle_error+1)\n\t"				// load our function address
			"str r7, [r6]\n\t"							// replace stacked PC

			"mov sp, r5\n\t"							// restore SP

			/*
			 * Idiotic code begin
			 * {
			 */
			"pop {r6, r7}\n\t"
			"mov r7, r6\n\t"							// restore r7 saved by interrupt

			"mov r6, #0xFFFFFFF9\n\t"					// Not using saved LR because it's possible that it'll return
														// us into privileged handler mode

			"push {r6}\n\t"								// push because we can't use mov pc, r6
			"pop {pc}\n\t"								// Start return sequence

			/*
			 * }
			 * Idiotic code end
			 */

		);
	}
}


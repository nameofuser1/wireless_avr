#include <stm32f10x.h>
#include "controller.h"
#include "common/logging.h"
#include "esp8266.h"
#include "system.h"
#include "soft_timers/HardwareTimer.h"
#include "soft_timers/SoftwareTimer.h"
#include "EspUpdater.h"
#include "err.h"

void CLOCK_init(void);
static void gpio_init(void);

extern HardwareTimerDriver SystemTimer;
extern uint32_t device_err;

uint8_t state = 0;
extern uint32_t enter_addr;

static void err_cb(void)
{
	//__disable_fault_irq();
	//__disable_irq();
	NVIC_DisableIRQ(TIM2_IRQn);

	asm(
		"ldr r5, =enter_addr\n\t"
		"ldr r5, [r5]\n\t"							// r5 points to lr (0x08 after interrupt SP)

		"add r6, r5, #0x08\n\t"						// r6 points to stacked r0
		"mov r7, #0x01\n\t"
		"str r7, [r6]\n\t"							// store error code

		"add r6, r6, #0x18\n\t"						// r6 points to PC address
		"ldr r7, =(handle_error+1)\n\t"				// load our function address
		"str r7, [r6]\n\t"							// replace stacked PC

		"mov sp, r5\n\t"							// restore SP
		"pop {r7, pc}\n\t"							// begin interrupt return sequence

/*		"add r5, r5, #0x1C\n\t"					// move to begin of stack

		"ldr r6, [r5], #-8\n\t"					// load xPSR and decrement r5 by 8 (passing pc)
		"push {r6}\n\t"							// push xPSR on top of stack

		"ldr r8, =(handle_error+1)\n\t"			// save address of our function into r8
		"push {r8}\n\t"							// replace PC with our function address

		"ldr r6, [r5], #-14\n\t"				// load lr and set r5 to r7 pushed by interrupt (next is LR)
		"push {r6}\n\t"							// push lr on top of stack

		"mov r0, #0\n\t"
		"push {r0}\n\t"
		"push {r0}\n\t"
		"push {r0}\n\t"
		"push {r0}\n\t"

		"mov r0, #1\n\t"						// DEVICE_CRC_ERROR
		"push {r0}\n\t"

		"mov r6, #0xFFFFFFF9\n\t"
		"mov pc, r6\n\t"
		//"ldr r7, [r6], #-4\n\t"					// Load r7 saved by interrupt
		//"ldr pc, [r6]\n\t"
		//"mov pc, r6\n\t"						// r6 still contains LR so we start exception return sequence

		 */
	);

}

/*
static void reset_pin_cb(void)
{
	if(!state)
	{
		GPIOA->BSRR |= GPIO_BSRR_BS8;
		state = 1;
	}
	else
	{
		GPIOA->BSRR |= GPIO_BSRR_BR8;
		state = 0;
	}
}*/


int main(void)
{
	volatile uint32_t v = enter_addr;
	if(v)
	{

	}

	CLOCK_init();
	__enable_fault_irq();
	__enable_irq();

	system_init();
	gpio_init();
	EspUpdater_Init(115200);
	LOGGING_SetLevel(LOG_INFO);

	NVIC_EnableIRQ(TIM2_IRQn);

	SoftwareTimer pin_timer;
	SoftwareTimer_Init(&pin_timer);
	SoftwareTimer_Add_cb(&pin_timer, err_cb);
	SoftwareTimer_Arm(&pin_timer, Timer_Repeat, 25);
	LOGGING_Info("Starting timer (^_^)");
	SystemTimer.AddTimer(&pin_timer);

	while(1)
	{
		LOGGING_Info("LOG WAITING");
		SystemTimer.Delay(25);
	}


	/*
	 * USART1 is used by printf so we have to be able to use
	 * usart1 interrupt inside another one. By default every
	 * interrupt has the highest priority. So we lower usart2
	 * and usart3 priorities.
	 *
	 * Also we make usart2 prio lower then usart3 as usart3 is
	 * responsible for communication with esp. Some speed up.
	 */
	NVIC_SetPriority(USART3_IRQn, 1);
	NVIC_SetPriority(USART2_IRQn, 2);

	/*
	 * I know, I know...
	 * But it's the best way at time
	 */
	esp_wait_ready:
		ESP8266_WaitForReady();

	CONTROLLER_init();

	while(1)
	{
		if(!ESP8266_Ready())
		{
			LOGGING_Debug("Esp disconnected");
			CONTROLLER_DeInit();
			goto esp_wait_ready;
		}

		CONTROLLER_perform_action();
	}

    return 0;
}


/*
 * *************************************
 * AHB  Freq = HCLK
 * APB2 Freq = PCLK2
 * APB1 Freq = PCLK1
 * *************************************
 */
void CLOCK_init(void)
{
    /* HCLK = SYSCLK / 1 = 72MHz */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK / 2 = 36MHz */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PCLK1 = HCLK / 2 = 36MHz */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
}


static void gpio_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	/* PB5. Push-Pull 2MHz. Network info. */
	GPIOB->CRL &= ~GPIO_CRL_CNF5;
	GPIOB->CRL |= GPIO_CRL_MODE5_1;
	GPIOB->BSRR |= GPIO_BSRR_BR5;

	/* PB4. Input pull down. ESP ready. */
	GPIOB->CRL &= ~GPIO_CRL_CNF4;
	GPIOB->CRL |= GPIO_CRL_CNF4_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE4;
	GPIOB->ODR &= ~GPIO_ODR_ODR4;

	/* PB3. Input pull-down. ESP ready. Doesn't work in some reason */
	GPIOB->CRL &= ~GPIO_CRL_CNF3;
	GPIOB->CRL |= GPIO_CRL_CNF3_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE3;
	GPIOB->ODR &= ~GPIO_ODR_ODR3;

	/* PA8(reset) 2MHz push-pull, high(no reset)*/
	GPIOA->CRH &= ~GPIO_CRH_CNF8;
	GPIOA->CRH |= GPIO_CRH_MODE8_1;
	GPIOA->BSRR |= GPIO_BSRR_BS8;

	/*
	 * External interrupt on line 4
	 * On GPIOB 4. Rising/falling edges;
	 */
	/*
	AFIO->EXTICR[2] |= AFIO_EXTICR2_EXTI4_PB;
	EXTI->IMR |= EXTI_IMR_MR4;
	EXTI->RTSR |= EXTI_RTSR_TR4;
	EXTI->FTSR |= EXTI_FTSR_TR4;
	*/

	NVIC_EnableIRQ(EXTI4_IRQn);
}







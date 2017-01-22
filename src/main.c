#include <stm32f10x.h>
#include "controller.h"
#include "common/logging.h"
#include "EspUpdater.h"
#include "esp8266.h"


void CLOCK_init(void);
static void gpio_init(void);

uint8_t state = 0;

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
	CLOCK_init();
	__enable_fault_irq();
	__enable_irq();

	gpio_init();

	/*
	 * We make usart2 prio lower then usart3 as usart3 is
	 * responsible for communication with esp.
	 */
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_SetPriority(USART3_IRQn, 0);
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_SetPriority(USART1_IRQn, 2);

	EspUpdater_Init(115200);
	LOGGING_SetLevel(LOG_INFO);
	LOGGING_Info("Wainting for esp");

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


	NVIC_EnableIRQ(EXTI4_IRQn);*/
}







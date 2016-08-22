#include <crc32.h>
#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include "esp8266.h"
#include "controller.h"
#include <stdio.h>
#include <inttypes.h>
#include <stm32f10x_crc.h>
#include <stm32f10x_exti.h>
#include "soft_timers/SoftwareTimer.h"
#include "soft_timers/SoftwareTimer2.h"

void CLOCK_init(void);
static void gpio_init(void);


int main(void)
{
	CLOCK_init();
	__enable_fault_irq();
	__enable_irq();


	gpio_init();

	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//1 ms
	SoftwareTimer2_start();

	USART1_init();
	USART_Cmd(USART1, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

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
			printf("Esp disconected\r\n");
			CONTROLLER_DeInit();
			goto esp_wait_ready;
		}

		ResultCode code = CONTROLLER_perform_action();
		ProgramState state = CONTROLLER_get_state();
		switch(code)
		{
			case NONE:
				break;

			case INITIAL_ERROR:
				printf("Initial error in state %d\r\n", state);
				break;

			case TIMEOUT:
				printf("Timeout error in state %d\r\n", state);
				break;

			case PROG_TYPE_ERROR:
				printf("Prog type error in state %d\r\n", state);
				break;
		}
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







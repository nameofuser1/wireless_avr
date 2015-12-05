#include <common/CircularBuffer.h>
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>
#include "spi.h"
#include "UART.h"
#include "avr_flasher.h"
#include "soft_timers/SoftwareTimer2.h"


void CLOCK_init(void);
CircularBuffer usart_buffer;
static bool send_pr(void);
static void gpio_init(void);

SoftwareTimer timer;

void gpio_switch(void)
{
	GPIOA->ODR ^= GPIO_ODR_ODR1;
}

int main(void)
{
	CLOCK_init();
	__enable_irq();
	gpio_init();
	CircularBuffer_alloc(&usart_buffer, 1024);

	USART1_init();
	USART_Cmd(USART1, ENABLE);

	USART3_init();
    USART_Cmd(USART3, ENABLE);

	NVIC_EnableIRQ(USART3_IRQn);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	SoftwareTimer2_init();
	SoftwareTimer2_set_duration(1);	//10 ms
	SoftwareTimer2_start();

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI_init(SPI1);
	SPI_enable(SPI1);

	SoftwareTimer wait_at_ready_timer;
	SoftwareTimer_init(&wait_at_ready_timer);
	uint8_t retries = 0;
	bool success = false;

	while(!success && retries < 15)
	{
		AVRFlasher_reset_enable();
		SoftwareTimer_arm(&wait_at_ready_timer, OnePulse, 25);
		SoftwareTimer_start(&soft_timer2, &wait_at_ready_timer);
		SoftwareTimer_wait_for(&wait_at_ready_timer);
		success = send_pr();
		retries++;

		if(!success)
		{
			AVRFlasher_reset_disable();
			SoftwareTimer_arm(&wait_at_ready_timer, OnePulse, 2);
			SoftwareTimer_start(&soft_timer2, &wait_at_ready_timer);
			SoftwareTimer_wait_for(&wait_at_ready_timer);
		}
	}

	printf(success ? "Successful\r\n" : "Failed\r\n");
	if(success)
	{
		AvrCommand command;
		command.b1 = AT16_RD_SIG_B1;
		command.b2 = AT16_RD_SIG_B2;
		command.b3 = AT16_RD_VENDOR_CODE;
		command.b4 = AT16_ANSWER_BYTE;
		volatile uint8_t answer = AVRFlasher_send_command(&command);
		printf("Read vendor code: 0x%02x\r\n", answer);
	}

	AVRFlasher_reset_disable();

	while(1)
	{
		if(!CircularBuffer_is_empty(&usart_buffer))
		{
			uint8_t data = *(uint8_t*)(CircularBuffer_get(&usart_buffer));
			USART_SendData(USART1, data);
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


void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		uint8_t data = (uint8_t)(USART_ReceiveData(USART3) & 0xFF);
		CircularBuffer_put(&usart_buffer, (void*)&data);
	}
}


static bool send_pr(void)
{
	bool res = true;
	uint8_t answer;

	SPI_write(SPI1, AT16_PROG_EN_B1);
	while(!(SPI1->SR & SPI_SR_RXNE));
	answer = SPI1->DR;
	SPI_write(SPI1, AT16_PROG_EN_B2);
	while(!(SPI1->SR & SPI_SR_RXNE));
	answer = SPI1->DR;
	SPI_write(SPI1, AT16_PROG_EN_B3);
	while(!(SPI1->SR & SPI_SR_RXNE));
	answer = SPI1->DR;
	if(answer != 0x53)
	{
		res = false;
	}
	SPI_write(SPI1, AT16_PROG_EN_B4);
	while(!(SPI1->SR & SPI_SR_RXNE));
	answer = SPI1->DR;
	return res;
}


static void gpio_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	GPIOA->CRL &= ~GPIO_CRL_CNF1;	//Push-pull
	GPIOA->CRL |= GPIO_CRL_MODE1_1;	//2MHz
	//GPIOA->BSRR |= GPIO_BSRR_BS3;	//No reset


	GPIOA->CRL &= ~GPIO_CRL_CNF3;	//Push-pull
	GPIOA->CRL |= GPIO_CRL_MODE3_1;	//2MHz
	GPIOA->BSRR |= GPIO_BSRR_BS3;	//No reset

	/* SCK alternate push-pull 50MHz */
	GPIOA->CRL &= ~GPIO_CRL_CNF5;
	GPIOA->CRL |= GPIO_CRL_CNF5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5;

	/* MOSI alternate push-pull 50MHz */
	GPIOA->CRL &= ~GPIO_CRL_CNF7;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->CRL |= GPIO_CRL_MODE7;

	/* NSS alternate push-pull 50MHz */
	GPIOA->CRL &= ~GPIO_CRL_CNF4;
	GPIOA->CRL |= GPIO_CRL_CNF4_1;
	GPIOA->CRL |= GPIO_CRL_MODE4;
	//should make it as 1?

	/* MISO input floating */
	GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL &= ~GPIO_CRL_MODE6;
	GPIOA->CRL |= GPIO_CRL_CNF6_0;
}












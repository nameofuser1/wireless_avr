/*
 * mcu.h
 *
 *  Created on: Feb 2, 2017
 *      Author: kript0n
 */

#ifndef SRC_MCU_H_
#define SRC_MCU_H_

#include <stm32f10x.h>

#define MCU_RESET_PORT			GPIOA
#define MCU_RESET_PIN			GPIO_Pin_8
#define MCU_RESET_ON()			MCU_RESET_PORT->BSRR |= GPIO_BSRR_BR8
#define MCU_RESET_OFF()			MCU_RESET_PORT->BSRR |= GPIO_BSRR_BS8


void MCU_Init(void);
void MCU_DeInit(void);


#endif /* SRC_MCU_H_ */

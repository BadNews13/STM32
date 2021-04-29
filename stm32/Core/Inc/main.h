/*
 * main.h
 *
 *  Created on: Apr 21, 2021
 *      Author: bad_n
 */

#ifndef MAIN_H_
#define MAIN_H_




#include "stm32f10x.h"


static __IO uint32_t SysTick_CNT = 0;	//	для системного таймера

void RCC_DeInit(void);			//	сбрасываем тактирование
void SetSysClockTo72(void);		//	настраиваем на 72 MHz
void GPIO_Init (void);			//	настраиваем порты

#define SYSCLOCK 72000000U
void SysTick_Init(void);		//	системный таймер для задержек
void SysTick_Handler(void);		//	обработчик прерывания системного счетчика
void delay_ms(uint32_t ms);		//	функция задержки базирующаяся на системном счетчике


#endif /* MAIN_H_ */

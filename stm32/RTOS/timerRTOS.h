
#ifndef timerRTOS_H_
#define timerRTOS_H_

void timer2_ini (void);
#include "stm32f10x.h"

extern "C" {
void TIM2_IRQHandler(void);
}

#endif /* timerRTOS_H_ */

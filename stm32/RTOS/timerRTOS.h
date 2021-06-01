
#ifndef timerRTOS_H_
#define timerRTOS_H_


#include "stm32f10x.h"


void timer2_ini (void);

extern "C" {
void TIM2_IRQHandler(void);
}

#endif /* timerRTOS_H_ */

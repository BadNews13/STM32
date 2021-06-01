
#ifndef timerBLINK_H_
#define timerBLINK_H_

#include "stm32f10x.h"

void timerBLINK_ini (void);
/*
extern "C" {
	void TIM3_IRQHandler(void);
}
*/
void TIM3_IRQHandler(void);

#endif /* timerBLINK_H_ */

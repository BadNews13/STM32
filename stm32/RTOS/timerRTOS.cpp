

#include <timerRTOS.h>

void timer2_ini (void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Тактирование таймера TIM2

	TIM2->PSC = 72000000 / 1000 - 1; 		//	1000 tick/sec				//	Настройка предделителя таймера
	TIM2->ARR = 5000;  						//	1 Interrupt/sec (1000/100)	//	Загружаем число миллисекунд в регистр автоперезагрузки
	TIM2->DIER |= TIM_DIER_UIE; 			//	Enable tim2 interrupt		//	Разрешаем прерывание при переполнении счетчика
	TIM2->CR1 |= TIM_CR1_CEN;   			//	Start count					//	Запускаем счет

	NVIC_EnableIRQ(TIM2_IRQn);  			//	Enable IRQ
}


void TIM2_IRQHandler(void)
{
	static uint8_t i=0;
	TIM2->SR &= ~TIM_SR_UIF; 										//	Clean UIF Flag
	if (1 == (i++ & 0x1))		{GPIOC->BSRR = GPIO_BSRR_BS13;}		//	установить нулевой бит		(выключить светодиод)
	else						{GPIOC->BRR = ( 1 << 13 );}			//	сбросить нулевой бит		(включить светодиод)


}


extern "C" void TIM2_IRQHandler()
{
/*
    // Этот обработчик может обслуживать разные прерывания, поэтому,
    // сначала выясняем причину прерывания, затем выполняем
    // соответствующий ситуации код.
    if(TIM1->SR&TIM_SR_UIF)
    {
        // Сбрасываем флаг записью 0, те биты регистра SR, в которые
        // запишем 1, не изменятся.
        TIM1->SR=~TIM_SR_UIF;

        // Выполняем действия...
    }
    */
}
/*
 * gpio.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <gpio.h>

#define INPUT_PULL_UP_DOWN 0x8 // используется для обоих вариантов
#define GPIO_BITS_MASK 0xF // маска для стирания битов конфигурации



GPIO::GPIO(GPIO_TypeDef *port){

	this->GPIOx = port;
// тактируем порт от шины APB1
	if ( port == GPIOA){

			RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
			return;
	}

	if ( port == GPIOB ){

			RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
			return;
	}

	if ( port == GPIOC ){
			RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
			return;
	}

	if ( port == GPIOD ){
			RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
			return;
	}

	if ( port == GPIOE ){
			RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
			return;

	}

return;

}

void GPIO::pinConf ( uint8_t pin_nomber, uint8_t pin_mode ){

	this->pin_m = pin_mode; // для методов set/reset Pin если используется альтернативная ф-я
	uint8_t offset; // смещение в регистре
	uint8_t mode;

	// если вход с подтяжкой меняем pin_mode
	if ( ( pin_mode == INPUT_PULL_UP ) || ( pin_mode == INPUT_PULL_DOWN ) )	{mode = INPUT_PULL_UP_DOWN;}
	else																	{mode = pin_mode;}				//	проверить другие дефайны (режимы работы выводов)
	if ( pin_nomber < 8 ){

		offset = pin_nomber * 4;
		this->GPIOx->CRL &= ~( GPIO_BITS_MASK << offset );
		this->GPIOx->CRL |= ( mode << offset );
	} // if
	else if ( pin_nomber > 7 ){

		offset = ( pin_nomber - 8 ) * 4;					//	(13-8) * 4 = 20
		this->GPIOx->CRH &= ~( GPIO_BITS_MASK << offset );	//	стереть 4 бита // (0xF << 20) - (bit_23, bit_22, bit_21, bit_20)
		this->GPIOx->CRH |= ( mode << offset );				//	записать 4 бита

	} // else

	// если режим пулл-ап ставим бит пина в регистре ODR  в 1
	if ( pin_mode == INPUT_PULL_UP ){

		GPIOx->ODR |= ( 1 << pin_nomber );

	}

	/* доп. условие.  если режим задан INPUT_PULL_DOWN то сбрасываем бит пина в 0
	 * нужно для исключения ситуации, когда сначала назначили режим INPUT_PULL_UP
	 * а потом где-то в программе  переназначили режим INPUT_PULL_DOWN. В этом случае в
	 * регистре ODR останется 1 и пин все равно будет работать как INPUT_PULL_UP
	 */
	if ( pin_mode == INPUT_PULL_DOWN ){

		GPIOx->ODR &= ~( 1 << pin_nomber );

	}
return;
} //pinConf


void GPIO::setPin( uint8_t pin_nomber ){

	// если пин сконфигурирован как альтернативная ф-я ничего не делаем
	// т.к. управление пином должно быть из альтернативной ф-и
	if ( ( this->pin_m == AF_PUSH_PULL) || ( this->pin_m == AF_OPEN_DRAIN ) ){

		return;

	}// if

	this->GPIOx->BSRR = ( 1 << pin_nomber );
return;
}

void GPIO::resetPin( uint8_t pin_nomber ){

	// если пин сконфигурирован как альтернативная ф-я ничего не делаем
	// т.к. управление пином должно быть из альтернативной ф-и
	if ( ( this->pin_m == AF_PUSH_PULL) || ( this->pin_m == AF_OPEN_DRAIN ) ){

		return;

	}// if

	this->GPIOx->BRR = ( 1 << pin_nomber );
return;
}




int GPIO::getPin ( uint8_t pin_nomber ){

	uint16_t mask;
	mask = ( 1<< pin_nomber);

	if ( (this->GPIOx->IDR) & mask) return 1;

	else return 0;
}

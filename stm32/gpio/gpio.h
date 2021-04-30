/*
 * gpio.h
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#ifndef GPIO_H_
#define GPIO_H_

//#include "stm32f103xb.h"
#include "stm32f10x.h"


//---------------inputs-------------------------------------------------
#define INPUT_FLOATING 0x4 		// вход без подтяжки							0100
#define INPUT_PULL_UP 0x7F 		// с подтяжкой к питанию
#define INPUT_PULL_DOWN 0xFF 	// с подтяжкой к "земле"
#define INPUT_ANALOG 0x0 		// аналоговый вход

//--------------outputs--------------------------------------------------
#define OUTPUT_OPEN_DRAIN 0x7 // выход открытый сток
#define OUTPUT_PUSH_PULL 0x3 	// выход тяни-толкай

//--------------altarnate function---------------------------------------
#define AF_PUSH_PULL 0xB 		// альтернативная ф-я с выходом тяни-толкай		1011
#define AF_OPEN_DRAIN 0xF 		// альтернативная функция с открытым стоком		1111

class GPIO {

	public:

	GPIO();							// конструктор 1
	GPIO( GPIO_TypeDef *port );		// конструктор 2

	void pinConf ( uint8_t pin_nomber, uint8_t pin_mode ); // режим работы пина
	void setPin( uint8_t pin_nomber ); // установить 1 на пине
	void resetPin( uint8_t pin_nomber ); // сбросить пин
	int getPin ( uint8_t pin_nomber ); // считываем состояние пина (reg. IDR)

//	virtual ~GPIO();

	private:

	GPIO_TypeDef *GPIOx;
	int pin_m;

	protected:
	void enablePORT(GPIO_TypeDef *port);


};

#endif /* GPIO_H_ */

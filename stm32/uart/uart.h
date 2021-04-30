/*
 * uart.h
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#ifndef UART_H_
#define UART_H_

 // мой код
#include <gpio.h>
#include "stm32f10x.h"

/*
//	работает
class UART : GPIO {
public:
	UART(GPIO_TypeDef *port);
	virtual ~UART();
};
*/


class UART : GPIO {

public:
	UART(USART_TypeDef *uart);
	virtual ~UART();


private:
	USART_TypeDef *USARTx;
	GPIO_TypeDef *GPIOx;

};




#endif /* UART_H_ */











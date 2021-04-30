/*
 * uart.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <uart.h>


/*
//	работает
UART::UART(GPIO_TypeDef *port):GPIO(port) {

}
*/



UART::UART(USART_TypeDef *uart):GPIO(GPIOx) {

	this->GPIOx = GPIOA;

	if (uart == USART1)	{this->GPIOx = GPIOA;}


	/*
	switch (uart)
	{
		case USART1:	{this->GPIOx = GPIOA;}		break;
		case USART2:	{this->GPIOx = GPIOA;}		break;
		case USART3:	{this->GPIOx = GPIOA;}		break;
	}
*/

}





UART::~UART() {
	// TODO Auto-generated destructor stub
}

// public gpio

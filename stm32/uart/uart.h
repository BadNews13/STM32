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




#define	USART1_TX_pin	9
#define	USART1_RX_pin	10

#define	USART2_TX_pin	2
#define	USART2_RX_pin	3

#define	USART3_TX_pin	10
#define	USART3_RX_pin	11

#define	USART1_TX_pin_remaped	6
#define	USART1_RX_pin_remaped	7

/*
//	работает
class UART : GPIO {
public:
	UART(GPIO_TypeDef *port);
	virtual ~UART();
};
*/
extern "C" {
//#include "stm32f10x.h"
void USART1_IRQHandler(void);
}


class UART : GPIO {

public:
	UART(USART_TypeDef *uart, uint32_t BaudRate);
	virtual ~UART();

	void DMA_TX_init();
	void DMA_RX_init();


private:
	USART_TypeDef 	*USARTx;
	GPIO_TypeDef 	*GPIOx;

};




#endif /* UART_H_ */











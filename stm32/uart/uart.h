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
#include <dma.h>		//	пока не получается запихнуть его в юарт, т.к. нужно ДВА! обьекта этого класса. Поэтому пока создаем как отдельный обьект внутри данного класса
#include "stm32f10x.h"




#define	USART1_TX_pin	9
#define	USART1_RX_pin	10

#define	USART2_TX_pin	2
#define	USART2_RX_pin	3

#define	USART3_TX_pin	10
#define	USART3_RX_pin	11

#define	USART1_TX_pin_remaped	6
#define	USART1_RX_pin_remaped	7

#define OUTPUT	1
#define INPUT	0

extern "C" {
void USART1_IRQHandler(void);		//	обработчик прерывания от USART1

void uart1_init(uint32_t BaudRate, uint8_t *tx_buf, uint8_t *rx_buf);
}



	class UART : GPIO, DMA {
public:
	UART(USART_TypeDef *uart, uint32_t BaudRate);
	virtual ~UART();

	void DMA_TX_init();
	void DMA_RX_init();

	void dma_init(uint8_t direct, uint8_t *buf);								//	конфигурирует канал DMA (исользовать для TX и RX)

	static void led_on(void);



private:
	USART_TypeDef 	*USARTx;

};




#endif /* UART_H_ */











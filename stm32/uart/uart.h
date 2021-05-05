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

struct usart
{

};

// нужны статические переменные чтобы работать в прерываннии
	volatile static uint8_t tx_write_index;			//	количество байт оптравленных в очередь (которые уже отправляются)
	volatile static uint8_t tx_counter;				//	количество байт, ожидающих отправку
	volatile static uint8_t DMA_TX_start_position;	//	позиция с которой начнут перенаправлять байты
	volatile static uint8_t DMA_TX_count;			//	сколько байт перенаправить





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
	void USART1_IRQHandler(void);				//	обработчик прерывания от USART1
	void DMA1_Channel5_IRQHandler(void);		//	rx complete
	void DMA1_Channel4_IRQHandler(void);		//	tx complete
	void uart1_init(uint32_t BaudRate, uint8_t *tx_buf, uint8_t *rx_buf);
//	void put_byte_UART_1(uint8_t c);
	void set_ptr_on_obj(uint16_t *_ptr);

}

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128




	class UART : GPIO, DMA {
public:
	UART();											//	конструктор 1
	UART(USART_TypeDef *uart, uint32_t BaudRate);	//	конструктор 2
	virtual ~UART();

	void DMA_TX_init();
	void DMA_RX_init();

	void dma_init(uint8_t direct, uint8_t *buf);								//	конфигурирует канал DMA (исользовать для TX и RX)

	/*static*/ void led_on(void);

	void put_byte_UART_1(uint8_t c);


	void init(void);
	void put_byte(uint8_t byte);
	void DMA_interrupt_exe(void);




	uint32_t F_CPU;									//	частота тактирования микроконтроллера
	uint32_t BaudRate;								//	скорость передачи данных по uart

	uint8_t tx_pin;
	uint8_t rx_pin;

	uint8_t *tx_buf;								//	указатель на буфер отправки
	uint8_t *rx_buf;								//	указатель на буфер приема

	uint8_t tx_buf_size;							//	размер буфера отправки
	uint8_t rx_buf_size; 							//	размер буфера приема



	USART_TypeDef 	*USARTx;

private:


};




#endif /* UART_H_ */











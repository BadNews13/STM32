/*
 * uart.h
 *
 *  Created on: Apr 21, 2021
 *      Author: bad_n
 */

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

void USART1_Init(void);
void USART1_IRQHandler(void);
void DMA1_Init (void);

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128

char rx_str[RX_BUFFER_SIZE];
char tx_str[TX_BUFFER_SIZE];

void put_byte_UART1(uint8_t c);

#ifdef __cplusplus
}
#endif

#endif /* UART_H_ */

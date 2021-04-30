/*
 * uart.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <uart.h>




/*
//	работает
UART::UART(USART_TypeDef *uart):GPIO() {
//	GPIOC->BSRR = GPIO_BSRR_BR13;		//сбросить нулевой бит
}
*/


UART::UART(USART_TypeDef *uart, uint32_t BaudRate):GPIO() {

	uint32_t F_CPU = 72000000;
//	uint32_t BaudRate = 115200;

	if (uart == USART1)								//	PA9(TX)		PA10(RX)	//	доступен ремап на PB6(TX) и PB(RX)
	{
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);	//	тактирование периферии UART
		this->enablePORT(GPIOA);						//	такитрование периферии GPIO
		this->pinConf(USART1_TX_pin, AF_PUSH_PULL);		//	альтернативная функция с выходом пуш-пул	AF_PUSH_PULL
		this->pinConf(USART1_RX_pin, INPUT_FLOATING); 	//	вход без подтяжки 							INPUT_FLOATING

		////USART needs to be in disabled state, in order to be able to configure some bits in CRx registers
		if(READ_BIT(USART1->CR1, USART_CR1_UE) != (USART_CR1_UE))
		{
			MODIFY_REG(USART1->CR1,
									USART_CR1_M | USART_CR1_PCE | USART_CR1_PS,
									USART_CR1_TE |USART_CR1_RE);
		}

		//Async Mode
		CLEAR_BIT	(USART1->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
		CLEAR_BIT	(USART1->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
		WRITE_REG	(USART1->BRR, F_CPU/BaudRate);										//BaudRate
		SET_BIT		(USART1->CR1, USART_CR1_UE);										//Enable
	}

	if (uart == USART2)								//	PA2(TX)		PA3(RX)
	{
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);	//	тактирование периферии UART
		this->enablePORT(GPIOA);
		this->pinConf(USART2_TX_pin, AF_PUSH_PULL);				//	альтернативная функция с выходом пуш-пул	AF_PUSH_PULL
		this->pinConf(USART2_RX_pin, INPUT_FLOATING); 			//	вход без подтяжки 							INPUT_FLOATING

		////USART needs to be in disabled state, in order to be able to configure some bits in CRx registers
		if(READ_BIT(USART2->CR1, USART_CR1_UE) != (USART_CR1_UE))
		{
			MODIFY_REG(USART2->CR1,
									USART_CR1_M | USART_CR1_PCE | USART_CR1_PS,
									USART_CR1_TE |USART_CR1_RE);
		}

		//Async Mode
		CLEAR_BIT	(USART2->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
		CLEAR_BIT	(USART2->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
		WRITE_REG	(USART2->BRR, F_CPU/(BaudRate*2));										//BaudRate (в два раза медленнее будет)
		SET_BIT		(USART2->CR1, USART_CR1_UE);											//Enable

	}

	if (uart == USART3)								//	PA2(TX)		PA3(RX)
	{
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);	//	тактирование периферии UART
		this->enablePORT(GPIOB);
		this->pinConf(USART3_TX_pin, AF_PUSH_PULL);				//	альтернативная функция с выходом пуш-пул	AF_PUSH_PULL
		this->pinConf(USART3_RX_pin, INPUT_FLOATING); 			//	вход без подтяжки 							INPUT_FLOATING

		////USART needs to be in disabled state, in order to be able to configure some bits in CRx registers
		if(READ_BIT(USART3->CR1, USART_CR1_UE) != (USART_CR1_UE))
		{
			MODIFY_REG(USART3->CR1,
									USART_CR1_M | USART_CR1_PCE | USART_CR1_PS,
									USART_CR1_TE |USART_CR1_RE);
		}

		//Async Mode
		CLEAR_BIT	(USART3->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
		CLEAR_BIT	(USART3->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
		WRITE_REG	(USART3->BRR, F_CPU/(BaudRate*2));										//BaudRate (в два раза медленнее будет)
		SET_BIT		(USART3->CR1, USART_CR1_UE);											//Enable
	}


//	NVIC_EnableIRQ(USART1_IRQn);	//USART1 interrupt Init
}








UART::~UART() {
	// TODO Auto-generated destructor stub
}

// public gpio




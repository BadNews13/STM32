/*
 * uart.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <uart.h>


 uint8_t rx_str_1[RX_BUFFER_SIZE];
 uint8_t tx_str_1[TX_BUFFER_SIZE];


 void (UART::*DMA_interrupt_exe_ptr)(void);	//	создаем указатель, который будет указывать на метод обработки прерывания от USART1
 UART *uart1_ptr;							//	создаем указатель, который будет указывать на обьект USART1



 UART::UART(): GPIO(), DMA()
 {

 }



UART::UART(USART_TypeDef *uart, uint32_t BaudRate) : GPIO(), DMA(){
	uint32_t F_CPU = 72000000;
	this->USARTx = uart;

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


		//	разрешим от данного модуля локальные прерывания – по заполнению приёмного буфера и по ошибке передачи данных
		SET_BIT(USART1->CR1, USART_CR1_RXNEIE);
		SET_BIT(USART1->CR3, USART_CR3_EIE);

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

		//	разрешим от данного модуля локальные прерывания – по заполнению приёмного буфера и по ошибке передачи данных
		SET_BIT(USART2->CR1, USART_CR1_RXNEIE);
		SET_BIT(USART2->CR3, USART_CR3_EIE);


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

		//	разрешим от данного модуля локальные прерывания – по заполнению приёмного буфера и по ошибке передачи данных
		SET_BIT(USART3->CR1, USART_CR1_RXNEIE);
		SET_BIT(USART3->CR3, USART_CR3_EIE);

	}

	NVIC_EnableIRQ(USART1_IRQn);	//USART1 interrupt Init
}


void UART::led_on(void)
{
	if (uart1_ptr->USARTx == USART1)	{GPIOC->BRR = ( 1 << 13 );}	//	сбросить нулевой бит	(включить светодиод)
}


UART::~UART() {
	// TODO Auto-generated destructor stub
}


void USART1_IRQHandler(void)
{
	if(		(READ_BIT(USART1->SR, USART_SR_RXNE) 		== 	(USART_SR_RXNE)) &&			//	Read data register not empty
			(READ_BIT(USART1->CR1, USART_CR1_RXNEIE)	== 	(USART_CR1_RXNEIE))		)	//	RXNE interrupt enable
	{
		uint8_t byte =  USART1->DR;
		USART1->DR = byte;
	}
}


void UART::dma_init(uint8_t direct, uint8_t *buf)
{
	this->enableDMA (DMA1);									//	включить тактирование шины на которой висит DMA

	switch (direct)
	{
		case OUTPUT:	{this->set_chanel(DMA1_Channel4);}	break;		//	выбираем 4-ый канал		//	(USART1_TX)
		case INPUT:		{this->set_chanel(DMA1_Channel5);}	break;		//	выбираем 5-ый канал		//	(USART1_RX)
	}

	this->disable_chanel();										//	выключаем канал
	WRITE_REG(this->CHANELx->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)
	WRITE_REG(this->CHANELx->CMAR, (uint32_t)&buf);				//	указываем куда делать транзакцию (в память)
	this->set_priority(VERY_HIGH);								//	ставим максимальный приоритет (для прерывания)
	this->set_peripheral_increment(DISABLE_MODE); 				//	выбираем режим инкрементации адреса для периферии
	this->set_memory_increment(ENABLE_MODE);					//	выбираем режим инкрементации адреса для памяти
	this->set_memory_data_width(SIZE_8_BITS);					//	устанавливаем размер передаваемого символа

	switch (direct)
	{
		case OUTPUT:
		{
			this->tx_buf = buf;
			this->set_direct(MEM_TO_PERIPHERAL);				//	устанавливаем направление транзакции
			this->set_circular_mode(DISABLE_MODE);				//	устанавливаем режим циклический/нормальный

			NVIC_EnableIRQ(DMA1_Channel4_IRQn);					//	DMA1_Channel4_IRQn interrupt init	(USART1_TX)
			SET_BIT	(this->USARTx->CR3, USART_CR3_DMAT);		//	Enable DMA Mode for transmission
		}
		break;

		case INPUT:
		{
			this->rx_buf = buf;
			this->set_direct(PERIPHERAL_TO_MEM);				//	устанавливаем направление транзакции
			this->set_circular_mode(ENABLE_MODE);				//	устанавливаем режим циклический/нормальный

			NVIC_EnableIRQ(DMA1_Channel5_IRQn);					//	DMA1_Channel5_IRQn interrupt init (USART1_RX)
			SET_BIT	(this->USARTx->CR3, USART_CR3_DMAR);		//	Enable DMA Mode for reception

			this->enable_chanel();								//	включаем канал
		}
		break;
	}

	this->reset_flag(ALL_FLAGS);				//	сбрасываем все флаги
	this->enable_interrupt(TRANSFER_COMPLETE);	//	разрешаем прерывания
//	this->enable_chanel();						//	включаем канал
}


void DMA1_Channel5_IRQHandler(void)	//	закончился прием от ПК (RX)
{
	GPIOC->BSRR = GPIO_BSRR_BR13;		//сбросить нулевой бит
	if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF5) == (DMA_ISR_TCIF5)) // если поднят флаг - завершена пересылка
	{
		WRITE_REG(DMA1->IFCR, DMA_IFCR_CTCIF5);					//	сбрасывем флаг записываю в него 1
		USART1->DR = rx_str_1[0];				//	для проверки отправим символ из массива в который писали
		GPIOC->BSRR = GPIO_BSRR_BR13;		//	сбросить нулевой бит (включим светодиод на плате)
	}
	else if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF5) == (DMA_ISR_TEIF5))	//	если поднят бит - ошибка передачи
	{
		CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);		//Disable DMA channels 4
		CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);		//Disable DMA channels 5
	}
}


void UART::put_byte_UART_1(uint8_t c)
{
	__disable_irq (); // запретить прерывания		(tx_counter может измениться в прерывании)
	// записываем байт в буфер
	tx_buf[tx_write_index++] = c;								//	запишем символ в строку (для DMA доступа)
	if (tx_write_index == tx_buf_size)	{tx_write_index = 0;}	//	если массив закончился, то переходим в начало
	tx_counter++;												//	увеличим количество байт ожидающих отправку
	__enable_irq ();  // разрешить прерывания		(лучше в конце всей функции поставить, но пока попробуем так)

	//	Если идет передача
	if (READ_REG(DMA_CNDTR4_NDT)&&READ_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN) == (DMA_CCR4_EN))//	если мы еще прошлые байты не отправили и если канал открыт
	{return;}							//	в прерывании по завершении перенаправления отработает этот вариант
	else								//	если DMA сейчас не перенаправляет байты
	{
		//	после того как мы положили байт и поняли что линия сейчас "свободна" мы должны передать байты в DMA_транзакцию
		//	если был переход через конец/начало буфера, то отправляем только до конца буфера, а продолжим в прерывании по завершении транзакции

		if(tx_counter > tx_write_index)		//	выполнен переход
		{
			if(tx_counter == 1)				//	переход на одном байте
			{
				DMA_TX_start_position = 	tx_buf_size - 1;	//	считаем стартовую позицию
				DMA_TX_count = 				1;					//	считаем сколько байт надо перенаправить сейчас
				tx_counter = 				0;					//	считаем остаток
			}
			else							//	переход на пакете
			{
				DMA_TX_start_position = 	tx_buf_size - (tx_counter - tx_write_index)-1;	//	считаем стартовую позицию	//	узнаем индекс начала пакета в буфере
				DMA_TX_count = 				tx_counter - tx_write_index-1;					//	только до конца буфера		// узнаем сколько байт лежит до конца буфера
				tx_counter = 				tx_counter - DMA_TX_count+1;						//	срезаем с счетчика отправлямое количество
			}
		}
		else								//	перехода не было
		{
			DMA_TX_start_position = 	tx_write_index - tx_counter;				//	считаем стартовую позицию
			DMA_TX_count = 				tx_counter;									//	считаем сколько байт надо перенаправить сейчас
			tx_counter = 				0;											//	считаем остаток
		}
		/*
		for (uint8_t i = 0; i<5; i++)	{}	//	нужна задержка если через вызов метода выполнять запуск DMA, иначе проскакивают двойные байты (кастыль)
		DMA_transaction(&tx_buf[DMA_TX_start_position], DMA_TX_count);
		*/

//	чтобы обойтись без кастыля, то используем код из этого метода
		//	Запускаем перенаправление
		CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_EN);							//	Disable DMA channel 4
		WRITE_REG	(DMA1_Channel4->CMAR, (uint32_t)&tx_buf[DMA_TX_start_position]);	//	указываем с какого места памяти делать транзакцию (в uart)
		//	указываем сколько байт надо перенаправить
		MODIFY_REG	(DMA1_Channel4->CNDTR,										//	Set Number of data to transfer
											DMA_CNDTR4_NDT,						//	сбросить оставшееся количетсво байт для передачи
											  	  	  	  	  DMA_TX_count);	//	записать это значение
		SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_EN);  						//	Enable DMA channel 4
	}
}



void UART::init(void)
{
	tx_write_index = 0;			//	количество байт оптравленных в очередь (которые уже отправляются)
	tx_counter = 0;				//	количество байт, ожидающих отправку
	DMA_TX_start_position = 0;	//	позиция с которой начнут перенаправлять байты
	DMA_TX_count = 0;			//	сколько байт перенаправить

	DMA_interrupt_exe_ptr = &UART::DMA_interrupt_exe;		//	глобальный указатель на метод класса (обработчик прерываний)

	if (USARTx == USART1)	{SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);}	//	тактирование периферии UART
	if (USARTx == USART2)	{SET_BIT(RCC->APB2ENR, RCC_APB1ENR_USART2EN);}	//	тактирование периферии UART
	if (USARTx == USART3)	{SET_BIT(RCC->APB2ENR, RCC_APB1ENR_USART3EN);}	//	тактирование периферии UART


	////USART needs to be in disabled state, in order to be able to configure some bits in CRx registers
	if(READ_BIT(USARTx->CR1, USART_CR1_UE) != (USART_CR1_UE))
	{
		MODIFY_REG(USARTx->CR1,
								USART_CR1_M | USART_CR1_PCE | USART_CR1_PS,
								USART_CR1_TE |USART_CR1_RE);
	}

	//Async Mode
	CLEAR_BIT	(USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
	CLEAR_BIT	(USARTx->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));

	if (USARTx == USART1)									//	PA9(TX)		PA10(RX)	//	доступен ремап на PB6(TX) и PB(RX)
	{
		enablePORT(GPIOA);									//	такитрование периферии GPIO
		WRITE_REG	(USART1->BRR, F_CPU/BaudRate);			//	BaudRate
	}

	if (USARTx == USART2)									//	PA2(TX)		PA3(RX)
	{
		enablePORT(GPIOA);									//	такитрование периферии GPIO
		WRITE_REG	(USART2->BRR, F_CPU/(BaudRate*2));		//	BaudRate (в два раза медленнее будет)
	}

	if (USARTx == USART3)									//	PA2(TX)		PA3(RX)
	{
		enablePORT(GPIOB);									//	такитрование периферии GPIO
		WRITE_REG	(USART3->BRR, F_CPU/(BaudRate*2));		//	BaudRate (в два раза медленнее будет)
	}

	//	разрешим от данного модуля локальные прерывания – по заполнению приёмного буфера и по ошибке передачи данных
	SET_BIT(USARTx->CR1, USART_CR1_RXNEIE);
	SET_BIT(USARTx->CR3, USART_CR3_EIE);
	SET_BIT(USARTx->CR1, USART_CR1_UE);						//	Enable

	this->pinConf(tx_pin, AF_PUSH_PULL);					//	альтернативная функция с выходом пуш-пул	AF_PUSH_PULL
	this->pinConf(rx_pin, INPUT_FLOATING); 					//	вход без подтяжки 							INPUT_FLOATING

	if (USARTx == USART1)	{NVIC_EnableIRQ(USART1_IRQn);}	//USART1 interrupt Init
	if (USARTx == USART2)	{NVIC_EnableIRQ(USART2_IRQn);}	//USART2 interrupt Init
	if (USARTx == USART3)	{NVIC_EnableIRQ(USART3_IRQn);}	//USART3 interrupt Init

}


void UART::DMA_interrupt_exe(void)
{
	if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF4) == (DMA_ISR_TCIF4))		//	если передали данные из памяти в периферию
	{
		// узнаем докинули ли нам еще байт пока отправляли
		// или же мы просто обрабатываем переход кольцевого буфера к началу
		if (tx_counter)
		{
			if (tx_write_index >= tx_counter)	//	перехода не было // пакет лежит в между началом и концом буфера
			{
				DMA_TX_start_position = 	tx_write_index - tx_counter;	//	считаем стартовую позицию
				DMA_TX_count = 				tx_counter;						//	считаем сколько байт надо перенаправить сейчас
				tx_counter = 				0;								//	считаем остаток
			}

			if (tx_write_index < tx_counter)
			{
				if(tx_counter == 1)				//	переход на одном байте
				{
					DMA_TX_start_position = 	tx_buf_size - 1;	//	считаем стартовую позицию
					DMA_TX_count = 				1;					//	считаем сколько байт надо перенаправить сейчас
					tx_counter = 				0;					//	считаем остаток
				}
				else							//	переход на пакете
				{
					DMA_TX_start_position = 	(tx_buf_size - (tx_counter - tx_write_index));	//	считаем стартовую позицию	//	узнаем индекс начала пакета в буфере
					DMA_TX_count = 				tx_counter - tx_write_index;					//	только до конца буфера		// узнаем сколько байт лежит до конца буфера
					tx_counter = 				tx_counter - DMA_TX_count;						//	срезаем с счетчика отправлямое количество
				}
			}

			//			DMA_transaction(&tx_buf[DMA_TX_start_position], DMA_TX_count);			//	отправляем

			//	чтобы обойтись без кастыля, то используем код из этого метода
			//	Запускаем перенаправление
			CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_EN);						//	Disable DMA channel 4
			WRITE_REG	(DMA1_Channel4->CMAR, (uint32_t)&tx_buf[DMA_TX_start_position]);	//	указываем с какого места памяти делать транзакцию (в uart)
			//	указываем сколько байт надо перенаправить
			MODIFY_REG	(DMA1_Channel4->CNDTR,									//	Set Number of data to transfer
												DMA_CNDTR4_NDT,					//	сбросить оставшееся количетсво байт для передачи
																DMA_TX_count);	//	записать это значение
			SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_EN);  					//	Enable DMA channel 4
		}
		else    {CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);}					//	Disable DMA channels 4 // если больше нечего отправлять
	}


	if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF4) == (DMA_ISR_TEIF4))	//	если ошибка передачи
	{
	//Disable DMA channels
		CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);
		CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);
	}

	WRITE_REG(DMA1->IFCR, DMA_IFCR_CTCIF4);				//	сбросим флаг этого прерывания
}




void DMA1_Channel4_IRQHandler(void)	//	закончилась отправка (TX)
{
	(uart1_ptr->*DMA_interrupt_exe_ptr)();
}
//----------------------------------------------------------

void set_ptr_on_obj(uint16_t *_ptr)
{
	uart1_ptr =  ((UART*)_ptr);	//	этот указатель будет ипсользован в обработчике прерывания (обьект нельзя использовать, т.к. он еще не создан)
}

void UART::DMA_transaction(uint8_t *buf, uint8_t cnt)
{
	//	Запускаем перенаправление
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_EN);					//	Disable DMA channel 4
	WRITE_REG	(DMA1_Channel4->CMAR, (uint32_t)&buf[0]);			//	указываем с какого места памяти делать транзакцию (в uart)
	//	указываем сколько байт надо перенаправить
	MODIFY_REG		(DMA1_Channel4->CNDTR,							//	Set Number of data to transfer
										  DMA_CNDTR4_NDT,			//	сбросить оставшееся количетсво байт для передачи
										  	  	  	  	  cnt);		//	записать это значение
	SET_BIT			(DMA1_Channel4->CCR, DMA_CCR4_EN);  			//	Enable DMA channel 4
}



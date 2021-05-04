/*
 * uart.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <uart.h>

volatile static uint8_t tx_write_index = 0;			//	количество байт оптравленных в очередь (которые уже отправляются)
volatile static uint8_t tx_counter = 0;				//	количество байт, ожидающих отправку

volatile static	uint8_t	DMA_TX_start_position = 0;	//	позиция с которой начнут перенаправлять байты
volatile static	uint8_t DMA_TX_count = 0;			//	сколько байт перенаправить

 uint8_t rx_str_1[RX_BUFFER_SIZE];
 uint8_t tx_str_1[TX_BUFFER_SIZE];


UART::UART(USART_TypeDef *uart, uint32_t BaudRate) : GPIO() {
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
	GPIOC->BRR = ( 1 << 13 );			//	сбросить нулевой бит		(включить светодиод)
}



UART::~UART() {
	// TODO Auto-generated destructor stub
}



void USART1_IRQHandler(void)
{

	UART::led_on();
	if(		(READ_BIT(USART1->SR, USART_SR_RXNE) 		== 	(USART_SR_RXNE)) &&			//	Read data register not empty
			(READ_BIT(USART1->CR1, USART_CR1_RXNEIE)	== 	(USART_CR1_RXNEIE))		)	//	RXNE interrupt enable
	{
		uint8_t byte =  USART1->DR;
		USART1->DR = byte;
	}
}


void uart1_init(uint32_t BaudRate, uint8_t *tx_buf, uint8_t *rx_buf)
{
	UART *uart1  = new UART(USART1, BaudRate);		//	создаем обьект класса UART
/*
	uart1->dma_init(OUTPUT, &tx_buf[0]);				//	внутри экземпляра создаем обьект DMA для отправки данных
	uart1->dma_init(INPUT, &rx_buf[0]);					//	внутри экземплеяра создаем обьект DMA для приема данных
*/
	uart1->dma_init(INPUT, &rx_str_1[0]);					//	внутри экземплеяра создаем обьект DMA для приема данных
	uart1->dma_init(OUTPUT, &tx_str_1[0]);				//	внутри экземпляра создаем обьект DMA для отправки данных


}




void UART::dma_init(uint8_t direct, uint8_t *buf)
{
	this->enableDMA (DMA1);									//	включить тактирование шины на которой висит DMA

	switch (direct)
	{
		case OUTPUT:
		{
			this->set_chanel(DMA1_Channel4);				//	выбираем 4-ый канал		//	(USART1_TX)
			this->set_direct(MEM_TO_PERIPHERAL);			//	устанавливаем направление транзакции
			this->set_circular_mode(DISABLE_MODE);			//	устанавливаем режим циклический/нормальный
//			this->set_peripheral_adr(&(USART1->DR));			//	указываем адрес периферии
			WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)
			WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&tx_str_1);			//	указываем куда делать транзакцию (в память)

			this->disable_chanel();							//	выключаем канал
			NVIC_EnableIRQ(DMA1_Channel4_IRQn);				//	DMA1_Channel4_IRQn interrupt init	(USART1_TX)
			SET_BIT	(this->USARTx->CR3, USART_CR3_DMAT);	//	Enable DMA Mode for transmission
		}
		break;

		case INPUT:
		{
			this->set_chanel(DMA1_Channel5);				//	выбираем 5-ый канал		//	(USART1_RX)
			this->set_direct(PERIPHERAL_TO_MEM);			//	устанавливаем направление транзакции
			this->set_circular_mode(ENABLE_MODE);			//	устанавливаем режим циклический/нормальный
//			this->set_peripheral_adr(uint8_t *peripheral);	//	указываем адрес периферии
			WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)

				WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&rx_str_1);			//	указываем куда делать транзакцию (в память)
			this->disable_chanel();							//	выключаем канал
			NVIC_EnableIRQ(DMA1_Channel5_IRQn);				//	DMA1_Channel5_IRQn interrupt init (USART1_RX)
			SET_BIT	(this->USARTx->CR3, USART_CR3_DMAR);	//	Enable DMA Mode for reception
		}
		break;
	}

	this->set_priority(VERY_HIGH);						//	ставим максимальный приоритет (для прерывания)
	this->set_peripheral_increment(DISABLE_MODE); 		//	выбираем режим инкрементации адреса для периферии
	this->set_memory_increment(ENABLE_MODE);			//	выбираем режим инкрементации адреса для памяти
	this->set_memory_data_width(SIZE_8_BITS);			//	устанавливаем размер передаваемого символа
//	this->set_memory_adr(buf);							//	указываем адрес памяти
	this->reset_flag(ALL_FLAGS);						//	сбрасываем все флаги
	this->enable_interrupt(TRANSFER_COMPLETE);			//	разрешаем прерывания
//	this->enable_chanel();								//	включаем канал

}



void DMA1_Channel4_IRQHandler(void)	//	закончилась отправка (TX)
{

  if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF4) == (DMA_ISR_TCIF4))		//	если передали данные из памяти в периферию
  {
    WRITE_REG(DMA1->IFCR, DMA_IFCR_CTCIF4);						//	сбросим флаг

    if (tx_counter)
    {
	  CLEAR_BIT		(DMA1_Channel4->CCR, DMA_CCR4_EN);						//	Disable DMA channel 4

	  DMA_TX_start_position = DMA_TX_start_position + DMA_TX_count;


	  if (DMA_TX_start_position > TX_BUFFER_SIZE)	{DMA_TX_start_position = 0;}
	  WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str_1[DMA_TX_start_position]);		//	указываем с какого места памяти делать транзакцию (в uart)

	  MODIFY_REG	(DMA1_Channel4->CNDTR,										//	Set Number of data to transfer
											  DMA_CNDTR4_NDT,					//	сбросить <~оставшееся~> количетсво байт для передачи
											  	  	  	  	  	  tx_counter);	//	записать это значение
	  SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_EN);  						//	Enable DMA channel 4
	  tx_counter = 0;
    }
    else    {CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);}					//	Disable DMA channels 4

  }
  else
  {
	if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF4) == (DMA_ISR_TEIF4))	//	если ошибка передачи
	{
		//Disable DMA channels
		CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);
		CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);
	}
  }

}
//----------------------------------------------------------

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

void put_byte_UART_1(uint8_t c)
{
	// записываем байт в буфер
	tx_str_1[tx_write_index++] = c;									//	запишем символ в строку (для DMA доступа)
	if (tx_write_index == TX_BUFFER_SIZE)	{tx_write_index = 0;}	//	если массив закончился, то переходим в начало
	tx_counter++;													//	увеличим количество байт ожидающих отправку

	//	Если идет передача
	if (/*READ_REG(DMA_CNDTR4_NDT)&&*/READ_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN) == (DMA_CCR4_EN))//	если мы еще прошлые байты не отправили и если канал открыт
	{return;}							//	в прерывании по завершении перенаправления отработает этот вариант
	else								//	если DMA сейчас не перенаправляет байты
	{
		CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);
		//	если пакет разделен концом буфера
		if ((tx_counter - tx_write_index) > 1)	// узнаем произошел ли разрыв пакета (определяем по количеству ожидающих в буфер байт и текщему индексу)
		{
			DMA_TX_start_position = 	TX_BUFFER_SIZE + tx_write_index - tx_counter;	//	считаем стартовую позицию
			DMA_TX_count = 				TX_BUFFER_SIZE - DMA_TX_start_position-1;		//	считаем сколько байт надо перенаправить сейчас
			tx_counter = 				tx_counter - DMA_TX_count;						//	считаем остаток
		}
		else
		{
			DMA_TX_start_position = 	tx_write_index - tx_counter;					//	считаем стартовую позицию
			DMA_TX_count = 				tx_counter;										//	считаем сколько байт надо перенаправить сейчас
			tx_counter = 				0;												//	считаем остаток
		}

		//	Запускаем перенаправление
		CLEAR_BIT		(DMA1_Channel4->CCR, DMA_CCR4_EN);								//	Disable DMA channel 4
		WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str_1[DMA_TX_start_position]);		//	указываем с какого места памяти делать транзакцию (в uart)
		//	указываем сколько байт надо перенаправить
		MODIFY_REG		(DMA1_Channel4->CNDTR,											//	Set Number of data to transfer
											  DMA_CNDTR4_NDT,							//	сбросить оставшееся количетсво байт для передачи
											  	  	  	  	  DMA_TX_count);			//	записать это значение
		SET_BIT			(DMA1_Channel4->CCR, DMA_CCR4_EN);  							//	Enable DMA channel 4

	}


}






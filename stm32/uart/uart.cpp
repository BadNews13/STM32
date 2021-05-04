/*
 * uart.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: bad_n
 */

#include <uart.h>





UART::UART(USART_TypeDef *uart, uint32_t BaudRate) : GPIO() {
//UART::UART(USART_TypeDef *uart, uint32_t BaudRate) : GPIO(), DMA() {
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


	NVIC_EnableIRQ(USART1_IRQn);	//USART1 interrupt Init
}


void UART::DMA_TX_init()
{
	//USART1_TX Init
	MODIFY_REG	(DMA1_Channel4->CCR, DMA_CCR4_MEM2MEM, DMA_CCR4_DIR);		//	Set transfer direction (Memory to Peripheral)
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PL);							//	Set priority level
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_CIRC);						//	Transfer mode NORMAL
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PINC);						//	Set peripheral no increment mode
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_MINC);						//	Set memory increment mode
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PSIZE_1 | DMA_CCR4_PSIZE_0);	//	Set peripheral data width
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_MSIZE_1 | DMA_CCR4_MSIZE_0);	//	Set memory data width

//	NVIC_EnableIRQ(USART1_IRQn);	//USART1 interrupt Init


	CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);			//Disable DMA channels 4


	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CGIF4);			//Clear Channel 4 global interrupt flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTCIF4);			//Clear Channel 4 transfer complete flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTEIF4);			//Clear Channel 4 transfer error flag


	SET_BIT		(USART1->CR3, USART_CR3_DMAT);			//Enable DMA Mode for transmission
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_TCIE);	//Enable Channel 4 Transfer complete interrupt
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_TEIE);	//Enable Channel 4 Transfer error interrupt



//	WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)
//	WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str);			//	указываем из какой памяти делать транзакцию (из uart)

}


void UART::DMA_RX_init()
{
	//USART1_RX Init
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_DIR | DMA_CCR5_MEM2MEM);		//	Set transfer direction (Peripheral to Memory)
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_PL);							//	Set priority level
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_CIRC);						//	Transfer mode NORMAL
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_PINC);						//	Set peripheral no increment mode
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_MINC);						//	Set memory increment mode
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_PSIZE_1 | DMA_CCR5_PSIZE_0);	//	Set peripheral data width
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_MSIZE_1 | DMA_CCR5_MSIZE_0);	//	Set memory data width

	CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);			//Disable DMA channels 5

	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CGIF5);			//Clear Channel 5 global interrupt flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTCIF5);			//Clear Channel 5 transfer complete flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTEIF5);			//Clear Channel 5 transfer error flag

	SET_BIT		(USART1->CR3, USART_CR3_DMAR);			//Enable DMA Mode for reception
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_TCIE);	//Enable Channel 5 Transfer complete interrupt
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_TEIE);	//Enable Channel 5 Transfer error interrupt

//	WRITE_REG(DMA1_Channel5->CPAR, (uint32_t)&(USART1->DR));	//	указываем из какой периферии делать транзакцию (из uart)
//	WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&rx_str);			//	указываем куда делать транзакцию (в память)
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

	uart1->dma_init(OUTPUT, &tx_buf[0]);				//	внутри экземпляра создаем обьект DMA для отправки данных
	uart1->dma_init(INPUT, &rx_buf[0]);					//	внутри экземплеяра создаем обьект DMA для приема данных
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








#include <uart_1.h>

volatile static uint8_t tx_write_index = 0;			//	количество байт оптравленных в очередь (которые уже отправляются)
volatile static uint8_t tx_counter = 0;				//	количество байт, ожидающих отправку

volatile static	uint8_t	DMA_TX_start_position = 0;	//	позиция с которой начнут перенаправлять байты
volatile static	uint8_t DMA_TX_count = 0;			//	сколько байт перенаправить

void USART1_Init(void)
{
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);										//USART Clock
	uint8_t	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);		(void) tmpreg;

	// настраиваем пины для uart
	MODIFY_REG(GPIOA->CRH,
							  GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1 | GPIO_CRH_CNF9_0,
							  GPIO_CRH_MODE9  | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF9_1);


	//USART1 DMA Init

	//USART1_RX Init
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_DIR | DMA_CCR5_MEM2MEM);		//	Set transfer direction (Peripheral to Memory)
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_PL);							//	Set priority level
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_CIRC);						//	Transfer mode NORMAL
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_PINC);						//	Set peripheral no increment mode
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_MINC);						//	Set memory increment mode
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_PSIZE_1 | DMA_CCR5_PSIZE_0);	//	Set peripheral data width
	CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_MSIZE_1 | DMA_CCR5_MSIZE_0);	//	Set memory data width

	//USART1_TX Init
	MODIFY_REG	(DMA1_Channel4->CCR, DMA_CCR4_MEM2MEM, DMA_CCR4_DIR);		//	Set transfer direction (Memory to Peripheral)
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PL);							//	Set priority level
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_CIRC);						//	Transfer mode NORMAL
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PINC);						//	Set peripheral no increment mode
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_MINC);						//	Set memory increment mode
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_PSIZE_1 | DMA_CCR4_PSIZE_0);	//	Set peripheral data width
	CLEAR_BIT	(DMA1_Channel4->CCR, DMA_CCR4_MSIZE_1 | DMA_CCR4_MSIZE_0);	//	Set memory data width


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
	WRITE_REG	(USART1->BRR, ((0x27)<<4)|0x01);										//115200
	SET_BIT		(USART1->CR1, USART_CR1_UE);											//Enable

	NVIC_EnableIRQ(USART1_IRQn);	//USART1 interrupt Init


	CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);			//Disable DMA channels 4
	CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);			//Disable DMA channels 5

	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CGIF4);			//Clear Channel 4 global interrupt flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTCIF4);			//Clear Channel 4 transfer complete flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTEIF4);			//Clear Channel 4 transfer error flag

	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CGIF5);			//Clear Channel 5 global interrupt flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTCIF5);			//Clear Channel 5 transfer complete flag
	WRITE_REG	(DMA1->IFCR, DMA_IFCR_CTEIF5);			//Clear Channel 5 transfer error flag

	SET_BIT		(USART1->CR3, USART_CR3_DMAT);			//Enable DMA Mode for transmission
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_TCIE);	//Enable Channel 4 Transfer complete interrupt
	SET_BIT		(DMA1_Channel4->CCR, DMA_CCR4_TEIE);	//Enable Channel 4 Transfer error interrupt

	SET_BIT		(USART1->CR3, USART_CR3_DMAR);			//Enable DMA Mode for reception
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_TCIE);	//Enable Channel 5 Transfer complete interrupt
	SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_TEIE);	//Enable Channel 5 Transfer error interrupt

	WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)
	WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str);			//	указываем из какой памяти делать транзакцию (из uart)

	WRITE_REG(DMA1_Channel5->CPAR, (uint32_t)&(USART1->DR));	//	указываем из какой периферии делать транзакцию (из uart)
	WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&rx_str);			//	указываем куда делать транзакцию (в память)


}


void USART1_IRQHandler(void)
{
	if(		(READ_BIT(USART1->SR, USART_SR_RXNE) 		== 	(USART_SR_RXNE)) &&			//	Read data register not empty
			(READ_BIT(USART1->CR1, USART_CR1_RXNEIE)	== 	(USART_CR1_RXNEIE))		)	//	RXNE interrupt enable
	{
		// открываем канал пересылки байта из периферии в память
		CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_EN);				//Disable DMA channel 5
		MODIFY_REG	(DMA1_Channel5->CNDTR, DMA_CNDTR5_NDT, 1);		//Set Number of data to transfer
		SET_BIT		(DMA1_Channel5->CCR, DMA_CCR5_EN);				//Enable DMA channel 5
	}
}


void DMA1_Init (void)
{
	uint32_t tmpreg;	//	пока используется для задержки.

	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);							  //DMA controller clock enable
	tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);  (void)tmpreg;

	NVIC_EnableIRQ(DMA1_Channel4_IRQn);		//DMA1_Channel4_IRQn interrupt init	(USART1_TX)
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);		//DMA1_Channel5_IRQn interrupt init (USART1_RX)
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
	  WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str[DMA_TX_start_position]);		//	указываем с какого места памяти делать транзакцию (в uart)

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
	USART1->DR = rx_str[0];				//	для проверки отправим символ из массива в который писали
	GPIOC->BSRR = GPIO_BSRR_BR13;		//	сбросить нулевой бит (включим светодиод на плате)
  }
  else if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF5) == (DMA_ISR_TEIF5))	//	если поднят бит - ошибка передачи
  {
    CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);		//Disable DMA channels 4
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR5_EN);		//Disable DMA channels 5
  }
}




void put_byte_UART1(uint8_t c)
{
	// записываем байт в буфер
	tx_str[tx_write_index++] = c;									//	запишем символ в строку (для DMA доступа)
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
		WRITE_REG(DMA1_Channel4->CMAR, (uint32_t)&tx_str[DMA_TX_start_position]);		//	указываем с какого места памяти делать транзакцию (в uart)
		//	указываем сколько байт надо перенаправить
		MODIFY_REG		(DMA1_Channel4->CNDTR,											//	Set Number of data to transfer
											  DMA_CNDTR4_NDT,							//	сбросить оставшееся количетсво байт для передачи
											  	  	  	  	  DMA_TX_count);			//	записать это значение
		SET_BIT			(DMA1_Channel4->CCR, DMA_CCR4_EN);  							//	Enable DMA channel 4

	}


}


/*
 * dma.cpp
 *
 *  Created on: 4 мая 2021 г.
 *      Author: bad_n
 */

// Название битов пока не буду перепиывать. Слишком их много получиться для 8-и каналов. Позиции битов для всех каналов совпадают

#include "dma.h"

DMA::DMA() {
	// TODO Auto-generated constructor stub

}

DMA::~DMA() {
	// TODO Auto-generated destructor stub
}

//	выбираем канал
void DMA::set_chanel(DMA_Channel_TypeDef *chanel)	//	если использовали пустой конструктор, то вызываем эту функцию, чтобы сконфигурировать порт
{
	this->CHANELx = chanel;	//	сохраняем канал, т.к. он нужен для дальнейшей работы

	if (this->CHANELx == DMA1_Channel1)	{}
	if (this->CHANELx == DMA1_Channel2)	{}
	if (this->CHANELx == DMA1_Channel3)	{}
	if (this->CHANELx == DMA1_Channel4)	{NVIC_EnableIRQ(DMA1_Channel4_IRQn);}	//DMA1_Channel4_IRQn interrupt init	(USART1_TX)
	if (this->CHANELx == DMA1_Channel5)	{NVIC_EnableIRQ(DMA1_Channel5_IRQn);}	//DMA1_Channel5_IRQn interrupt init (USART1_RX)
	if (this->CHANELx == DMA1_Channel6)	{}
	if (this->CHANELx == DMA1_Channel7)	{}

}



//	выбираем направление (кто отправляет, а кто получает)
void DMA::set_direct(uint8_t dir)
{
	switch (dir)
	{
		case PERIPHERAL_TO_MEM:
		{
			CLEAR_BIT	(this->CHANELx->CCR, DMA_CCR5_DIR | DMA_CCR5_MEM2MEM);		//	Set transfer direction (Peripheral to Memory)
		}
		break;

		case MEM_TO_PERIPHERAL:
		{
			MODIFY_REG	(this->CHANELx->CCR, DMA_CCR4_MEM2MEM, DMA_CCR4_DIR);		//	Set transfer direction (Memory to Peripheral)
		}
		break;

		case MEM_TO_MEM:	{}	break;
	}
}


//	устанавливаем приоритет
void DMA::set_priority(uint8_t priority_level)
{
	switch (priority_level)
	{
		case LOW:		{}	break;
		case MEDIUM:	{}	break;
		case HIGH:		{}	break;

		case VERY_HIGH:
		{
			CLEAR_BIT	(this->CHANELx->CCR, DMA_CCR5_PL);							//	Set priority level
		}
		break;
	}
}

//	устанавливаем режим циклический/нормальный
void DMA::set_circular_mode(uint8_t mode)
{
	switch (mode)
	{
		case DISABLE_MODE:
		{
			SET_BIT		(this->CHANELx->CCR, DMA_CCR5_CIRC);						//	Transfer mode NORMAL
		}
		break;

		case ENABLE_MODE: {} break;
	}
}

//	выбираем режим инкрементации адреса для периферии
void DMA::set_peripheral_increment(uint8_t mode)
{
	switch (mode)
	{
		case DISABLE_MODE:
		{
			CLEAR_BIT	(this->CHANELx->CCR, DMA_CCR5_PINC);						//	Set peripheral no increment mode
		}
		break;

		case ENABLE_MODE:		{}		break;

	}
}

//	выбираем режим инкрементации адреса для памяти
void DMA::set_memory_increment(uint8_t mode)
{
	switch (mode)
	{
		case DISABLE_MODE:		{}		break;

		case ENABLE_MODE:
		{
			SET_BIT		(this->CHANELx->CCR, DMA_CCR4_MINC);						//	Set memory increment mode
		}
		break;
	}
}

//	устанавливаем размер передаваемого символа
void DMA::set_memory_data_width(uint8_t width)
{
	switch (width)
	{
		case SIZE_8_BITS:
		{
			CLEAR_BIT	(DMA1_Channel5->CCR, DMA_CCR5_MSIZE_1 | DMA_CCR5_MSIZE_0);	//	Set memory data width
		}
		break;

		case SIZE_16_BITS:		{}		break;
		case SIZE_32_BITS:		{}		break;
	}
}

//	включаем канал
void DMA::enable_chanel(void)	//	если использовали пустой конструктор, то вызываем эту функцию, чтобы сконфигурировать порт
{
	SET_BIT	(this->CHANELx->CCR, DMA_CCR4_EN);  		//	Enable DMA channel
}

//	выключаем канал
void DMA::disable_chanel(void)
{
	CLEAR_BIT(this->CHANELx->CCR, DMA_CCR4_EN);		//Disable DMA channels 4
}

//	сбрасываем все флаги
void DMA::reset_flag(uint8_t flag)
{
	switch (flag)
	{
		case ALL_FLAGS:
		{
			WRITE_REG	(this->DMAx->IFCR, DMA_IFCR_CGIF4);			//Clear Channel 4 global interrupt flag
			WRITE_REG	(this->DMAx->IFCR, DMA_IFCR_CTCIF4);			//Clear Channel 4 transfer complete flag
			WRITE_REG	(this->DMAx->IFCR, DMA_IFCR_CTEIF4);			//Clear Channel 4 transfer error flag
		}
		break;

		case GLOBAL_INTERRUPT:
		{
			WRITE_REG	(this->DMAx->IFCR, DMA_IFCR_CGIF4);			//Clear Channel 4 global interrupt flag
		}
		break;

		case TRANSFER_COMPLETE:
		{
			WRITE_REG	(this->DMAx->IFCR, DMA_IFCR_CTCIF4);			//Clear Channel 4 transfer complete flag
		}
		break;

		case TRANSFER_ERROR:
		{
			WRITE_REG	(DMAx->IFCR, DMA_IFCR_CTEIF4);			//Clear Channel 4 transfer error flag
		}
		break;
	}

}


//	разрешаем прерывания
void DMA::enable_interrupt(uint8_t interrupt)
{
	switch (interrupt)
	{
		case TRANSFER_COMPLETE:
		{
			SET_BIT		(CHANELx->CCR, DMA_CCR4_TCIE);	//Enable Channel 4 Transfer complete interrupt
		}
		break;
		case TRANSFER_ERROR:
		{
			SET_BIT		(CHANELx->CCR, DMA_CCR4_TEIE);	//Enable Channel 4 Transfer error interrupt
		}
		break;
	}
}


//	указываем адрес периферии
void DMA::set_peripheral_adr(uint8_t *peripheral)
{
//	WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(USART1->DR));	//	указываем в какую периферию делать транзакцию (в uart)
	WRITE_REG(DMA1_Channel4->CPAR, (uint32_t)&(peripheral));	//	указываем в какую периферию делать транзакцию (в uart)
}


//	указываем адрес памяти
void DMA::set_memory_adr(uint8_t *memory)
{
//	WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&rx_str);			//	указываем куда делать транзакцию (в память)
	WRITE_REG(DMA1_Channel5->CMAR, (uint32_t)&memory);			//	указываем куда делать транзакцию (в память)
}


//	включить тактирование шины на которой висит DMA
void DMA::enableDMA (DMA_TypeDef *dma)
{
	DMA:DMAx = dma;	//	сохраняем канал, т.к. он нужен для дальнейшей работы

	if (dma == DMA1)	{SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);}	//DMA controller clock enable
//	if (dma == DMA2)	{SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);}	//DMA controller clock enable

	uint32_t tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);  (void)tmpreg;
}



/*
 * spi_2.c
 *
 *  Created on: Jun 25, 2021
 *      Author: bad_n
 */



#include <spi_2.h>


void SPI2_Init (void)
{
	SPI2_GPIO_Init();

	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);	  										//RCC peripheral clock enabling
	uint8_t tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN); 	(void) tmpreg;	  //Delay after an RCC peripheral clock enabling

//	WRITE_REG	(SPI2->CR1,	SPI_CR1_BR & ( SPI_CR1_BR_2 |  SPI_CR1_BR_1 |  SPI_CR1_BR_0));	//	111	/256	BaudRate (самый медленный)
	uint8_t BaudRate = 0;
	SET_BIT		(BaudRate, (1<<SPI_CR1_BR_2));
	SET_BIT		(BaudRate, (1<<SPI_CR1_BR_1));
	SET_BIT		(BaudRate, (1<<SPI_CR1_BR_0));
	WRITE_REG	(SPI2->CR1,	BaudRate);

	CLEAR_BIT 	(SPI2->CR1, SPI_CR1_CPHA);			//	Bit 0 CPHA:			Clock phase
	CLEAR_BIT 	(SPI2->CR1, SPI_CR1_CPOL);			//	Bit 1 CPOL:			Clock polarity							0: CK to 0 when idle
	SET_BIT 	(SPI2->CR1, SPI_CR1_MSTR);			//	Bit 2 MSTR:			Master selection						1: Master configuration

	CLEAR_BIT	(SPI2->CR1, SPI_CR1_SPE);			//	Bit 6 SPE:			SPI enable								0: Peripheral disabled
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_LSBFIRST);		//	Bit 7 LSBFIRST:		Frame format							0: MSB transmitted first
	SET_BIT		(SPI2->CR1, SPI_CR1_SSI);			//	Bit 8 SSI:			Internal slave select
	SET_BIT		(SPI2->CR1, SPI_CR1_SSM);			//	Bit 9 SSM:			Software slave management
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_RXONLY);		//	Bit 10 RXONLY:		Receive only							0: Full duplex (Transmit and receive)
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_DFF);			//	Bit 11 DFF:			Data frame format						0: 8-bit data frame format is selected for transmission/reception
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_CRCNEXT);		//	Bit 12 CRCNEXT:		CRC transfer next						0: Data phase (no CRC phase)
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_CRCEN);			//	Bit 13 CRCEN:		Hardware CRC calculation enable			0: CRC calculation disabled
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_BIDIOE);		//	Bit 14 BIDIOE:		Output enable in bidirectional mode		0: Output disabled (receive-only mode) in bidirectional mode
	CLEAR_BIT	(SPI2->CR1, SPI_CR1_BIDIMODE);		//	Bit 15 BIDIMODE:	Bidirectional data mode enable			0: 2-line unidirectional data mode selected

//	uint8_t rx_byte = SPI1->DR;						//	чтобы с флагами разобраться
//	rx_byte = READ_REG(SPI1->SR);

	SET_BIT		(SPI2->CR1, SPI_CR1_SPE);			//	SPI enable
	//WRITE_REG(SPI2->DR, 0xA2);					//	отправить байт в SPI
}




void SPI2_GPIO_Init (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	uint8_t offset;

	// PB13 - SCK2
	offset = (pin_SPI2_SCK - 8) * 4;							//	5 * 4 = 20
	GPIOB->CRH &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита
	GPIOB->CRH |= ( AF_PUSH_PULL << offset );			//	записать 4 бита
	//GPIOA->BSRR = ( 1 << pin_SPI1_SCK );				//	установка линии в 1
	//GPIOA->BRR = ( 1 << pin_SPI1_SCK );				//	установка линии в 0

	// PB14 - MISO2
	offset = (pin_SPI2_MISO - 8) * 4;							//	6 * 4 = 24
	GPIOB->CRH &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита
	GPIOB->CRH |= ( INPUT_FLOATING << offset );			//	записать 4 бита
	GPIOB->BSRR = ( 1 << pin_SPI2_MISO );				//	установка линии в 1
	//GPIOA->BRR = ( 1 << pin_SPI1_MISO );				//	установка линии в 0

	// PB15 - MOSI2
	offset = (pin_SPI2_MOSI - 8) * 4;							//	7 * 4 = 28
	GPIOB->CRH &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита
	GPIOB->CRH |= ( AF_PUSH_PULL << offset );			//	записать 4 бита
	GPIOB->BSRR = ( 1 << pin_SPI2_MOSI );				//	установка линии в 1
	//GPIOA->BRR = ( 1 << pin_SPI1_MOSI );				//	установка линии в 0


}

uint8_t SPI2_put_byte (uint8_t tx_byte)
{
	SPI2->DR = tx_byte;
	while(READ_BIT(SPI2->SR, SPI_SR_BSY));
	uint8_t rx_byte = SPI2->DR;
	return rx_byte;
}








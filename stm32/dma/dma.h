/*
 * dma.h
 *
 *  Created on: 4 мая 2021 г.
 *      Author: bad_n
 */

#ifndef DMA_H_
#define DMA_H_

#include "stm32f10x.h"

#define PERIPHERAL_TO_MEM	1
#define MEM_TO_PERIPHERAL	2
#define MEM_TO_MEM			3

#define LOW					0
#define MEDIUM				1
#define HIGH				2
#define VERY_HIGH			3

#define DISABLE_MODE		0
#define ENABLE_MODE			1

#define SIZE_8_BITS			0
#define SIZE_16_BITS		1
#define SIZE_32_BITS		2

#define	ALL_FLAGS			0
#define	GLOBAL_INTERRUPT	1
#define	TRANSFER_COMPLETE	2
#define	TRANSFER_ERROR		3





class DMA {
public:

	DMA();									// конструктор 1
	DMA( DMA_Channel_TypeDef *chanel );		// конструктор 2
	virtual ~DMA();


	void enableDMA (DMA_TypeDef *dma);				//	включить тактирование шины на которой висит DMA

//	void set_chanel(DMA_Channel_TypeDef *chanel);	//	выбираем канал

	void set_direct(uint8_t dir);
	void set_priority(uint8_t priority_level);

	void set_circular_mode(uint8_t mode);			//	устанавливаем режим циклический/нормальный
	void set_peripheral_increment(uint8_t mode); 	//	выбираем режим инкрементации адреса для периферии
	void set_memory_increment(uint8_t mode);		//	выбираем режим инкрементации адреса для памяти
	void set_memory_data_width(uint8_t width);		//	устанавливаем размер передаваемого символа

	void set_peripheral_adr(uint8_t *peripheral);	//	указываем адрес периферии
	void set_memory_adr(uint8_t *memory);			//	указываем адрес памяти

	void reset_flag(uint8_t flag);					//	сбрасываем все флаги
	void enable_interrupt(uint8_t interrupt);		//	разрешаем прерывания

	void enable_chanel(void);						//	включаем канал
	void disable_chanel(void);						//	выключаем канал

	void set_chanel(DMA_Channel_TypeDef *chanel);	//	выбираем канал

private:
	DMA_Channel_TypeDef *CHANELx;
	DMA_TypeDef			*DMAx;

protected:


};

#endif /* DMA_H_ */

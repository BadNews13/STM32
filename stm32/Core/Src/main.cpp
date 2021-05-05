/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <main.hpp>
#include <gpio.h>
#include <uart.h>
extern "C" {
#include "../../uart_1/uart_1.h"
}


int main(void)
{

	RCC_DeInit();		//	сбрасываем тактирование
	SetSysClockTo72();	//	тактирование от внешнего 8 MHz -> 72 MHz
	GPIO_Init();		//	настройка портов
	SysTick_Init();		//	запуск системного таймера (для функции delay)

	GPIOC->BSRR = GPIO_BSRR_BS13;		//установить нулевой бит

// Пример настройки светодиода на отладочной плате
GPIO *port = new GPIO(GPIOC); 				//	создаем экземпляр класса, передаем порт GPIOC
port->pinConf(13, OUTPUT_PUSH_PULL); 		//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port->setPin(13); 							//	установка вывода в 1
//port->resetPin(13); 						//	сброс вывода
/*
int value;
value = port->getPin (13); // считываем состояние вывода
*/

//UART *uart1 = new UART(USART1, 115200);
//UART *uart2 = new UART(USART2, 115200);
//UART *uart3 = new UART(USART3, 115200);


uint8_t tx_buf[30];
uint8_t rx_buf[30];

UART *uart1  = new UART();

uart1->USARTx = USART1;
uart1->F_CPU = 72000000;
uart1->BaudRate = 115200;

uart1->tx_pin = 9;
uart1->rx_pin = 10;

uart1->rx_buf = &rx_buf[0];
uart1->rx_buf_size = 30;

uart1->tx_buf = &tx_buf[0];
uart1->tx_buf_size = 30;

uart1->init();

set_ptr_on_obj((uint16_t*)uart1);	//	передаем указатель на обьект в класс (для обработки прерывания через вызов метода)


//for(uint8_t t = 0; t < 15; t++)		{uart1->put_byte_UART_1(t);}

delay_ms(100);
USART1->DR = 0x48;
USART2->DR = 0x76;
USART3->DR = 0x25;

//GPIOC->BSRR = GPIO_BSRR_BS13;		//	установить нулевой бит		(выключить светодиод)
//GPIOC->BRR = ( 1 << 13 );			//	сбросить нулевой бит		(включить светодиод)
uint8_t i = 0;
	while(1)
	{
/*
		port->setPin(13); 					// установка вывода в 1
		delay_ms(300);
		port->resetPin(13); 				// сброс вывода
		delay_ms(300);

		uart1->put_byte_UART_1(i++);
		uart1->put_byte_UART_1(i++);
		uart1->put_byte_UART_1(i++);
		uart1->put_byte_UART_1(i++);
*/

	}
}





void RCC_DeInit(void)
{
	SET_BIT(RCC->CR, RCC_CR_HSION);							//	Включим для начала HSI (внутренний генератор 8 МГц)
	while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == RESET)) {}		//	Дождёмся его стабилизации

	MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);				//	Сбросим калибровку

	CLEAR_REG(RCC->CFGR);									//	Полностью очистим конфигурационный регистр
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}	//	Дождёмся очистку бита SWS

	CLEAR_BIT(RCC->CR, RCC_CR_PLLON);						//	Отключим PLL
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}	//	Ждем отключения

	CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);		//	Выключим HSE и его детектор тактового сигнала
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}	//	Ждем отключения

	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);						//	Сбросим бит, разрешающий использование внешнего генератора

	SET_BIT(RCC->CSR, RCC_CSR_RMVF);						//	Сбросим флаги всех прерываний от RCC

	CLEAR_REG(RCC->CIR);									//	Также запретим все прерывания от RCC
}



void SetSysClockTo72(void)
{
  SET_BIT	(RCC->CR, RCC_CR_HSEON);						//	Включим наш HSE, дождавшись его стабилизации (HSI - вунтренняя RC цепочка))
  while(READ_BIT(RCC->CR, RCC_CR_HSERDY == RESET)) {}		//	Дождёмся его стабилизации

  CLEAR_BIT	(FLASH->ACR, FLASH_ACR_PRFTBE);					//	Disable the Prefetch Buffer
  SET_BIT	(FLASH->ACR, FLASH_ACR_PRFTBE);					//	Enable the Prefetch Buffer (так надо)

  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);	//	выбираем максимальную задержку

  // AHB prescaler											//	0xxx: SYSCLK not divided
  CLEAR_BIT	(RCC->CFGR, RCC_CFGR_HPRE_0);
  CLEAR_BIT	(RCC->CFGR, RCC_CFGR_HPRE_1);
  CLEAR_BIT	(RCC->CFGR, RCC_CFGR_HPRE_2);
  CLEAR_BIT	(RCC->CFGR, RCC_CFGR_HPRE_3);

  // APB low-speed prescaler (APB1)							//	100: HCLK divided by 2
  CLEAR_BIT	(RCC->CFGR,   RCC_CFGR_PPRE1_0);
  CLEAR_BIT	(RCC->CFGR,   RCC_CFGR_PPRE1_1);
  SET_BIT	(RCC->CFGR,   RCC_CFGR_PPRE1_2);

  // APB high-speed prescaler (APB2)						//	0xx: HCLK not divided
  CLEAR_BIT	(RCC->CFGR,  RCC_CFGR_PPRE2_0);
  CLEAR_BIT	(RCC->CFGR,  RCC_CFGR_PPRE2_1);
  CLEAR_BIT	(RCC->CFGR,  RCC_CFGR_PPRE2_2);

  // PLL entry clock source
  SET_BIT	(RCC->CFGR,  RCC_CFGR_PLLSRC);					//	1: HSE oscillator clock selected as PLL input clock

  // HSE divider for PLL entry
  CLEAR_BIT	(RCC->CFGR,  RCC_CFGR_PLLXTPRE);				//	0: HSE clock not divided

  // PLL multiplication factor								//	0111: PLL input clock x 9
  SET_BIT	(RCC->CFGR,  RCC_CFGR_PLLMULL_0);
  SET_BIT	(RCC->CFGR,  RCC_CFGR_PLLMULL_1);
  SET_BIT	(RCC->CFGR,  RCC_CFGR_PLLMULL_2);
  CLEAR_BIT	(RCC->CFGR,  RCC_CFGR_PLLMULL_3);

  SET_BIT(RCC->CR, RCC_CR_PLLON);									//	Разрешим работу PLL
  while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) != (RCC_CR_PLLRDY)) {}		//	Дождёмся его включения

  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);				//	Выберем PLL в качестве источника системного тактирования
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}	//	ждем применения данного действия

}



void GPIO_Init (void)
{
	uint32_t tmpreg;	//	пока используется для задержки.
	(void) tmpreg;
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);				//	Alternate function IO clock enable	(запуск тактирования для SWD отладчика
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);	//	Delay after an RCC peripheral clock enabling

	CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
	SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);		//	NOJTAG: JTAG-DP Disabled and SW-DP Enabled	(для отладчика)

	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);				//	enable port A	(запуск тактирование порта A)
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);	//	Delay after an RCC peripheral clock enabling



/*
	// Для примера включим светодиод на выводе А9
	// CNF 00: General purpose output Push-pull
	CLEAR_BIT	(GPIOA->CRH, GPIO_CRH_CNF9_1);
	CLEAR_BIT	(GPIOA->CRH, GPIO_CRH_CNF9_0);
	// MODE 11:	Output mode, max speed 50 MHz
	SET_BIT	(GPIOA->CRH, GPIO_CRH_MODE9_1);
	SET_BIT	(GPIOA->CRH, GPIO_CRH_MODE9_0);

	CLEAR_BIT	(GPIOA->BSRR, GPIO_ODR_ODR9);	//	выставляем на выводе лог. 0
	// конец конфигурирования вывода A9
*/
/*
	// Для примера включим светодиод на выводе А5
	// CNF 00: General purpose output Push-pull
	CLEAR_BIT	(GPIOA->CRL, GPIO_CRL_CNF5_1);
	CLEAR_BIT	(GPIOA->CRL, GPIO_CRL_CNF5_0);
	// MODE 11:	Output mode, max speed 50 MHz
	SET_BIT	(GPIOA->CRL, GPIO_CRL_MODE5_1);
	SET_BIT	(GPIOA->CRL, GPIO_CRL_MODE5_0);

	SET_BIT	(GPIOA->BSRR, GPIO_ODR_ODR5);		//	выставляем на выводе лог. 1 (так мы его выключим)

*/


	/*
	//===================================================
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);				//	enable port C	(запуск тактирование порта A)
		tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);	//	Delay after an RCC peripheral clock enabling

		// Для примера включим светодиод на выводе С13
		// CNF 00: General purpose output Push-pull
		CLEAR_BIT	(GPIOC->CRH, GPIO_CRH_CNF13_1);
		CLEAR_BIT	(GPIOC->CRH, GPIO_CRH_CNF13_0);
		// MODE 11:	Output mode, max speed 50 MHz
		SET_BIT	(GPIOC->CRH, GPIO_CRH_MODE13_1);
		SET_BIT	(GPIOC->CRH, GPIO_CRH_MODE13_0);

		GPIOC->BSRR = GPIO_BSRR_BR13;		//сбросить нулевой бит
//		GPIOC->BSRR = GPIO_BSRR_BS13;		//установить нулевой бит

 */

}

void SysTick_Init(void)
{

	  MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk,SYSCLOCK / 1000 - 1);
	  CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	  SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

}

extern "C" void SysTick_Handler(void)	//	обработчик прерывания системного счетчика
{
	if(SysTick_CNT > 0)  SysTick_CNT--;
}

void delay_ms(uint32_t ms)
{
	MODIFY_REG(SysTick->VAL,SysTick_VAL_CURRENT_Msk,SYSCLOCK / 1000 - 1);
	SysTick_CNT = ms;
	while(SysTick_CNT) {}
}



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
#include <rtos.h>

void led_on(void);
void motor (void);


extern "C" {
	#include <uart_1.h>
	#include <delay_ms.h>
	#include <delay_us.h>
	#include <timerBLINK.h>
	#include <lcd.h>
	#define TIM1_BRK_TIM15_IRQn TIM1_BRK_IRQn
	#define TIM1_BRK_IRQn 24
}

//UART * init_uart1(USART_TypeDef *USARTx, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t BaudRate);





#define TIM1_CH1_PA         8
#define TIM1_CH2_PA         9

#define PWM_VALUE          143		//	(144) 20		//	ширина импульса		50 	( не должно быть больше половины частоты, иначе будет сквозной ток)
#define TMR_T               287		//	200		//	частота				287 = 125kHz

#define DEADTIME            20		//	20

#define PP_MODE
//#define COMPL_MODE
void init_PP_MODE (void);


int main(void)
{
	RCC_DeInit();		//	сбрасываем тактирование
	SetSysClockTo72();	//	тактирование от внешнего 8 MHz -> 72 MHz
	GPIO_Init();		//	настройка портов
	SysTick_Init();		//	запуск системного таймера (для функции delay_ms())

	LCD_init();				//	инициализирует LCD1602

	//timerBLINK_ini();		//	запускаем мигание светодиода на отладочной плате

	RTOS_Init();						//	запускает RTOS
	RTOS_SetTask(led_on, 10000, 0);		// для теста (через ~10 секунд включится светодиод на отладочной плате)
	//RTOS_DeleteTask(led_on);

	uart1_init(9600);

	//init_PP_MODE();		//	генерация двух противофазных П-образных сигнала на частоте 125kHz


	GPIOC->BSRR = GPIO_BSRR_BS13;		//установить нулевой бит

// Пример настройки светодиода на отладочной плате
GPIO *port_C = new GPIO(GPIOC); 				//	создаем экземпляр класса, передаем порт GPIOC
port_C->pinConf(13, OUTPUT_PUSH_PULL); 		//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port_C->setPin(13); 							//	установка вывода в 1
//port_C->resetPin(13); 						//	сброс вывода
/*
int value;
value = port->getPin (13); // считываем состояние вывода
*/



/*
//	чтобы видеть жива ли плата controller_v1 (это пин TX1 (PA9))
//	тактирование шины A уже включено
uint8_t offset = ( 9 - 8 ) * 4;						//	(9-8) * 4 = 4
GPIOA->CRH &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита // (0xF << 20) - (bit_23, bit_22, bit_21, bit_20)
GPIOA->CRH |= ( OUTPUT_PUSH_PULL << offset );		//	записать 4 бита
*/


//	чтоБы видеть жива ли плата controller_v1 (это пин TX2 (PA2))
//	тактирование шины A уже включено
uint8_t offset = 2 * 4;								//	2 * 4 = 8
GPIOA->CRL &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита // (0xF << 20) - (bit_23, bit_22, bit_21, bit_20)
GPIOA->CRL |= ( OUTPUT_PUSH_PULL << offset );		//	записать 4 бита
GPIOA->BSRR = ( 1 << 2 );							//	установка линии в 1 (диод не светится)
//GPIOA->BRR = ( 1 << 2 );							//	установка линии TX2 в 0 (диод светится)

//	чтоБы видеть жива ли плата controller_v1 (это пин TX3 (PB10))

RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
offset = ( 10 - 8 ) * 4;							//	(10-8) * 4 = 28
GPIOB->CRH &= ~( GPIO_BITS_MASK << offset );		//	стереть 4 бита // (0xF << 20) - (bit_23, bit_22, bit_21, bit_20)
GPIOB->CRH |= ( OUTPUT_PUSH_PULL << offset );		//	записать 4 бита
GPIOB->BSRR = ( 1 << 10 );							//	установка линии в 1 (диод не светится)
//GPIOB->BRR = ( 1 << 10 );							//	установка линии TX3 в 0 (диод светится)



put_byte_UART1(0x66);
	while(1)
	{
		delay_ms(500);


		LCD_Command(0x01);		//	очистка дисплея					(LCD_CLEAR)
		delay_ms(2);			//	долгая операция
		LCD_Command(LCD_SETDDRAMADDR | 0);	//	писать с нулевого адреса
		LCDsendString(&uart1_rx_buf[0]);
/*
		GPIOA->BSRR = ( 1 << 2 );		// установка линии в 1
		delay_ms(300);
	//	GPIOA->BRR = ( 1 << 2 );		// установка линии в 0
		delay_ms(300);
*/
	/*
		port_C->setPin(13); 					// установка вывода в 1
		delay_ms(300);
		port_C->resetPin(13); 				// сброс вывода
		delay_ms(300);
	 */


		put_byte_UART1(0x01);
		put_byte_UART1(0x02);
		put_byte_UART1(0x03);
		put_byte_UART1(0x04);

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
}



/*
UART * init_uart1(USART_TypeDef *USARTx, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t BaudRate)
{
	UART *uart  = new UART();

	uart->USARTx = USARTx;
	uart->F_CPU = 72000000;
	uart->BaudRate = BaudRate;

	if	(USARTx == USART1)	{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	uart->tx_pin = 9;	uart->rx_pin = 10;}
	if	(USARTx == USART2)	{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	uart->tx_pin = 2;	uart->rx_pin = 3;}
	if	(USARTx == USART3)	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	uart->tx_pin = 2;	uart->rx_pin = 3;}

	uart->rx_buf = &rx_buf[0];
	uart->rx_buf_size = 30;

	uart->tx_buf = &tx_buf[0];
	uart->tx_buf_size = 30;

	uart->init();

	uart->dma_init(OUTPUT, &tx_buf[0]);			//	внутри экземпляра создаем обьект DMA для отправки данных
	uart->dma_init(INPUT, &rx_buf[0]);			//	внутри экземплеяра создаем обьект DMA для приема данных

	set_ptr_on_obj((uint16_t*)uart);			//	передаем указатель на обьект в класс (для обработки прерывания через вызов метода)

	return uart;
}

*/



extern "C" void TIM1_UP_TIM16_IRQHandler()
{
    // Этот обработчик может обслуживать разные прерывания, поэтому,
    // сначала выясняем причину прерывания, затем выполняем
    // соответствующий ситуации код.
    if(TIM1->SR&TIM_SR_UIF)
    {
        // Сбрасываем флаг записью 0, те биты регистра SR, в которые
        // запишем 1, не изменятся.
        TIM1->SR=~TIM_SR_UIF;

        // Выполняем действия...
    }
}



void init_PP_MODE (void)
{
    RCC->APB2ENR|=RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_TIM1EN;
/*
    GPIOA->CRH = SET_CRH(TIM1_CH1_PA,M_OUT_50M,OUT_AF_PP) | SET_CRH(TIM1_CH2_PA,M_OUT_50M,OUT_AF_PP);
    GPIOB->CRH = SET_CRH(TIM1_CH1N_PB,M_OUT_50M,OUT_AF_PP);
*/

    GPIO *port_A = new GPIO(GPIOA); 				//	создаем экземпляр класса, передаем порт GPIOC

    port_A->pinConf(TIM1_CH1_PA, AF_PUSH_PULL); 		//	задаем режим выход пуш-пул
    port_A->setPin(TIM1_CH1_PA); 							//	установка вывода в 1

    port_A->pinConf(TIM1_CH2_PA, AF_PUSH_PULL); 		//	задаем режим выход пуш-пул
    port_A->setPin(TIM1_CH2_PA); 							//	установка вывода в 1


#ifdef PP_MODE
    //CH1: PWM mode 2, CH2: PWM mode 1, preload enabled on all channels
    TIM1->CCMR1=
    		TIM_CCMR1_OC1M_2 |		//	Bit 6	OC1M: 	Output Compare 1 mode
			TIM_CCMR1_OC1M_1 |		//	Bit 5	OC1M: 	Output Compare 1 mode
			TIM_CCMR1_OC1M_0 |		//	Bit 4	OC1M: 	Output Compare 1 mode
			TIM_CCMR1_OC1PE |		//	Bit 3	OC1PE:	Output Compare 1 preload enable
			TIM_CCMR1_OC2M_2 |		//	Bit 14	OC2M:	Output Compare 2 mode
			TIM_CCMR1_OC2M_1 |		//	Bit 13	OC2M:	Output Compare 2 mode
			TIM_CCMR1_OC2PE;		//	Bit 11 	OC2PE:	Output Compare 2 preload enable

    TIM1->CCER=
    		TIM_CCER_CC1E |			//	Bit 0	CC1E:	Capture/Compare 1 output enable	(выводит сигнал на пин)
			TIM_CCER_CC2E;			//	Bit 4	CC2E:	Capture/Compare 2 output enable

    TIM1->BDTR=	TIM_BDTR_MOE;		//	Bit 15	MOE:	Main output enable

    TIM1->CCR1=	TMR_T - PWM_VALUE;	//	до куда считает канал 1
    TIM1->CCR2=	PWM_VALUE;			//	до куда считает канал 2

    TIM1->ARR=	TMR_T;				//	значение на котором перезагружаем отсчет

    TIM1->CR1=
    		TIM_CR1_ARPE |			//	Bit 7	ARPE:	Auto-reload preload enable
			TIM_CR1_CMS_1 |			//	Bits 6	CMS:	Center-aligned mode selection (выравнивание по центру)
			TIM_CR1_CMS_0;			//	Bits 6	CMS:	Center-aligned mode selection

    TIM1->CR1|=	TIM_CR1_CEN;			//	Bit 0	CEN:	Counter enable	(обязательно в последнюю очеред записывать этот бит)

    TIM1->EGR=	TIM_EGR_UG;			//	Bit 0	UG:		Update generation	(обновляем регситры)
#endif

#ifdef COMPL_MODE
    //CH1: PWM mode with complementary output & deadtime
    TIM1->CCMR1=TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM1->CCER=TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM1->BDTR=TIM_BDTR_MOE | DEADTIME;
    TIM1->CCR1=PWM_VALUE;
    TIM1->ARR=TMR_T;
    TIM1->CR1=TIM_CR1_ARPE;
    TIM1->CR1|=TIM_CR1_CEN;
    TIM1->EGR=TIM_EGR_UG;
#endif
}


void led_on(void)
{
	GPIOC->BRR = ( 1 << 13 );
}

void motor (void)
{

	/*
GPIO *port_A = new GPIO(GPIOA); 				//	создаем экземпляр класса, передаем порт GPIOA
port_A->pinConf(0, OUTPUT_PUSH_PULL); 			//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port_A->resetPin(0); 							//	установка вывода в 1

port_A->pinConf(1, OUTPUT_PUSH_PULL); 			//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port_A->resetPin(1); 							//	установка вывода в 1

port_A->pinConf(2, OUTPUT_PUSH_PULL); 			//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port_A->resetPin(2); 							//	установка вывода в 1

port_A->pinConf(3, OUTPUT_PUSH_PULL); 			//	задаем режим выход пуш-пул	OUTPUT_PUSH_PULL
port_A->resetPin(3); 							//	установка вывода в 1
*/
/*
//	1000	1100	0100	0110	0010	0011	0001	1001
//	0123	0123	0123	0123	0123	0123	0123	0123

	uint8_t i = 1;
	uint8_t k = 1;

	port_A->setPin(0);			//	1001
	delay_ms(k);

	port_A->resetPin(3);		//	1000
	delay_ms(i);

	port_A->setPin(1);			//	1100
	delay_ms(k);

	port_A->resetPin(0);		//	0100
	delay_ms(i);

	port_A->setPin(2);			//	0110
	delay_ms(k);

	port_A->resetPin(1);		//	0010
	delay_ms(i);

	port_A->setPin(3);			//	0011
	delay_ms(k);

	port_A->resetPin(2);		//	0001
	delay_ms(i);
	*/
}

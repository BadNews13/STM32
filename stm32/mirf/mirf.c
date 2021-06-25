//#include "../defines/defines_global.h"
//#include <util/delay.h>

#include "mirf.h"
#include "nRF24L01.h"
#include "spi_1.h"
#include "exti.h"
#include <delay_us.h>
#include <delay_ms.h>


#include "uart_2.h"	//	��� �������


//extern			uint8_t		MIRF_pack[MAX_PACK_LENGTH];
uint8_t in_data_to_mirf_for_test[mirf_PAYLOAD];

void NRF_CE (uint8_t value)
{
	switch (value)
	{
		case HIGH:	{ GPIOB->BSRR = ( 1 << NRF_CE_pin ); }	break;	//	поднимаем линию
		case LOW:	{ GPIOB->BRR = ( 1 << NRF_CE_pin );	}	break;	//	опускаем лини
	}
}

void NRF_CSN (uint8_t value)
{
	switch (value)
	{
		case HIGH:	{delay_us(1); 	GPIOB->BSRR = ( 1 << NRF_CS_pin ); 				}	break;	//	поднимаем линию
		case LOW:	{ 				GPIOB->BRR = ( 1 << NRF_CS_pin );	delay_us(1);}	break;	//	опускаем лини
	}
}

void NRF_SET_TX(void)
{
	//NRF_CE(LOW);										//	в ноль ( в этом режиме импульс (10 мкс) на CE запускает отправку из буфера FIFO)

	// Переход возможен только из режима Standby-I c задержкой 130us
	uint8_t tmp_config = NRF_read_reg(CONFIG);			//	получим текущие настройки из чипа
/*
	SET_BIT(tmp_config,PWR_UP);							//	включим питание антены
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	delay_ms(2);
*/

	CLEAR_BIT(tmp_config,PRIM_RX);						//	переведем в режим передатчика
//	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	delay_ms(1);										//	время для выхода антены в штатный режим работы

//	SET_BIT(tmp_config,PWR_UP);							//	включим питание антены
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	delay_ms(2);





//	NRF_CE(HIGH);										//	в ноль ( в этом режиме импульс (10 мкс) на CE запускает отправку из буфера FIFO)
}

void NRF_SET_RX(void)
{
	uint8_t tmp_config = NRF_read_reg(CONFIG);			//	получим текущие настройки из чипа
//	SET_BIT(tmp_config,PWR_UP);							//	включим питание антены
	SET_BIT(tmp_config,PRIM_RX);						//	переведем в режим приемника
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	NRF_CE(HIGH);										//	в режиме RX значение на CE(high) запускает прослушивание эфира
	delay_us(150);										//	время для выхода антены в штатный режим работы
}

void NRF_SET_PD(void)
{
	uint8_t tmp_config = NRF_read_reg(CONFIG);			//	получим текущие настройки из чипа
	CLEAR_BIT(tmp_config,PWR_UP);						//	выключим питание антены
	SET_BIT(tmp_config,PRIM_RX);						//	переведем в режим передатчика
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	NRF_CE(LOW);
}

void NRF_SET_PowerUp(void)
{

	uint8_t tmp_config = NRF_read_reg(CONFIG);	//	получим текущие настройки из чипа
	SET_BIT(tmp_config,PWR_UP);
	NRF_write_reg(CONFIG, tmp_config);
	delay_ms(2);

/*
	// if not powered up then power up and wait for the radio to initialize
	uint8_t tmp_config = NRF_read_reg(CONFIG);	//	получим текущие настройки из чипа
	if (!(tmp_config & (1<<PWR_UP)))	{SET_BIT(tmp_config,PWR_UP);}
	NRF_write_reg(CONFIG, tmp_config);
	// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
	delay_ms(2);
	*/
}

void NRF_cmd(uint8_t cmd)
{
	NRF_CSN(LOW);
//	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & cmd));
	SPI1_put_byte(cmd);
	NRF_CSN(HIGH);
}

//write many registers
void NRF_write_buf(uint8_t cmd, uint8_t *buf, uint8_t cnt)
{
	NRF_CSN(LOW);
	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & cmd));
	for (uint8_t i = 0; i < cnt; i++)	{SPI1_put_byte(buf[i]);}
	NRF_CSN(HIGH);
}

//read many registers
void NRF_read_buf(uint8_t reg, uint8_t *buf, uint8_t cnt)
{
    NRF_CSN(LOW);
    SPI1_put_byte(R_REGISTER | (REGISTER_MASK & reg));
	for (uint8_t i = 0; i < cnt; i++)    {buf[i] = SPI1_put_byte(0xFF);}
    NRF_CSN(HIGH);
}

//write one register
void NRF_write_reg (uint8_t adr, uint8_t data)
{
	NRF_CSN(LOW);											//	опускаем линию (начинается общение)
	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & adr));		//	отправляем команду
	SPI1_put_byte(data);									//	отправляем данные и сохраняем ответ
	NRF_CSN(HIGH);											//	поднимаем линию (общение окончено)
}

//read one register
uint8_t NRF_read_reg (uint8_t adr)
{
	uint8_t rx_byte;										//	для ответа
	NRF_CSN(LOW);											//	опускаем линию (начинается общение)
						 rx_byte = SPI1_put_byte(adr);		//	отправляем адрес регистра
	if (adr != 0xFF)	{rx_byte = SPI1_put_byte(0xFF);}	//	если ответ в следующем байте
	NRF_CSN(HIGH);											//	поднимаем линию (общение окончено)
	return rx_byte;
}

//initialize mirf
void NRF_Init(void)
{
	SPI1_Init();		//	для общения с чипом нужен SPI
	GPIO_NRF_Init();	//	инициализация остальных пинов к которым подключен модуль
	EXTI_Init();		//	инициализация внешнего прерывания, для реакции на прерывание от модуля

#ifdef MIRF_Master
	uint8_t self_adr[5] = 	{0xC7, 0xC7, 0xC7, 0xC7, 0xC7};
	uint8_t remote_adr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
#else
	uint8_t self_adr[5] = 	{0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	uint8_t remote_adr[5] = {0xC7, 0xC7, 0xC7, 0xC7, 0xC7};
#endif


	NRF_CE(LOW);					//	CE: Chip Enable. Зависит от режима работы. Если чип сконфигурен как приемник, то высокий (HIGH) уровень на CE позволяет чипу мониторить среду и получать пакеты. Низкий (LOW) уровень переводит чип в Standby-I и такая возможность становится уже недоступна. Если чип настроен на передачу, CE всегда держится на низком уровне. В этом случае для передачи данных нужно положить их в очередь FIFO и дернуть CE минимум на 10мкс (LOW->HIGH, 10мкс, HIGH->LOW).
	NRF_CSN(HIGH);					//	поднимаем линию (общение окончено) //	активный уровень - низкий (после того как пообщались - переводим в высокий) (при инициалисзации обязательно к +)

	delay_ms(15);		//	при подачи общего питания на чип нужно выждать паузу (10,3ms)

	NRF_write_reg(CONFIG, NRF_read_reg(CONFIG) & ~(1<<PWR_UP));	// выключим питание (регистры все равно будут доступны)


//====================================================================================================================================
/*
	// Настраиваем регистр CONFIG (0x00)
	uint8_t config_reg_value = NRF_read_reg(CONFIG);	//	переменная для настроек регистра CONFIG

	// -//-												//	7 bit:	не используется
	CLEAR_BIT	(config_reg_value, (1<<MASK_RX_DR));	//	6 bit:	если 0, то разрешает прерывание
	CLEAR_BIT	(config_reg_value, (1<<MASK_TX_DS));	//	5 bit:	если 0, то разрешает прерывание
	CLEAR_BIT	(config_reg_value, (1<<MASK_MAX_RT));	//	4 bit:	если 0, то разрешает прерывание
	SET_BIT		(config_reg_value, (1<<EN_CRC));		//	3 bit:	включаем расчет контрольной суммы
	SET_BIT		(config_reg_value, (1<<CRCO));			//	2 bit:	настраивает размер (поле) CRC - 2 байта
	CLEAR_BIT	(config_reg_value, (1<<PWR_UP));		//	1 bit:	питание пока держим выключенным
	CLEAR_BIT	(config_reg_value, (1<<PRIM_RX));		//	0 bit:	пока настраиваем как передатчик, т.к. в таком режиме меньшее энергопотребление

	NRF_write_reg(CONFIG, config_reg_value);	    // Reset NRF_CONFIG and enable 16-bit CRC. (включить 16-и битную контрольную сумму)

	put_byte_UART2(0xF1);	put_byte_UART2(NRF_read_reg(CONFIG));
*/

//====================================================================================================================================
/*
	// Настраиваем регистр EN_AA (0x01)								//	Включает автоподтверждение приема
	NRF_write_reg(EN_AA, 0);										//	выключим все автоподтверждения
	NRF_write_reg(EN_AA, NRF_read_reg(EN_AA) | (1 << ENAA_P0));		//	по нулевому каналу (трубе) будем отправлять подтверждение приема данных
	NRF_write_reg(EN_AA, NRF_read_reg(EN_AA) | (1 << ENAA_P1));		// 	если так, то по битам кажется, что отправка проходит успешно (при отсутствии принимающего устройства)
*/

	NRF_write_reg(EN_AA, (1 << ENAA_P1)); 							// включение автоподтверждения только по каналу 1
//	NRF_write_reg(EN_AA, (1 << ENAA_P0));

	put_byte_UART2(0xF2);	put_byte_UART2(NRF_read_reg(EN_AA));

//====================================================================================================================================
/*
	// Настраиваем регистр EN_RXADDR (0x02)			//	Выбирает активный канал приемника
	NRF_write_reg(EN_RXADDR, 0x00);					//	выключим все принимающие каналы

	NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P1));	// включим прием по 1-у каналу (основные данные)
	NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P0));	// включим прием по 1-у каналу (для приема ACK)
*/

	NRF_write_reg(EN_RXADDR, 0x00);					//	выключим все принимающие каналы
	NRF_write_reg(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); 		// включение каналов 0 и 1


	put_byte_UART2(0xF3);	put_byte_UART2(NRF_read_reg(EN_RXADDR));


//====================================================================================================================================

	// Настраиваем регистр SETUP_AW (0x03)				//	Задаем длину поля адреса
	NRF_write_reg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); 	// выбор длины адреса 5 байт

//====================================================================================================================================

	//	Настраиваем регистр SETUP_RETR (0x04)		//	Настройка параметров автоматического повтора отправки

	uint8_t delay_time = 0;		//	при 0 - время задержки перед повторной отправке 250ms
	uint8_t retry_cnt = 3;		//	повтор дважды

	NRF_write_reg(SETUP_RETR, delay_time | retry_cnt);

//====================================================================================================================================

	//	Настраиваем регистр RF_CH (0x05)	Задаем номер радиоканала
	NRF_write_reg(RF_CH, 5);	//124 (117)		//transmission channel	//	выбор радиоканала

//====================================================================================================================================

	//	Настройка регистра RF_SETUP (0x06)		//	Задаёт настройки радиоканала.
	uint8_t rf_setup_reg_value = 0;

	CLEAR_BIT	(rf_setup_reg_value, (1<<CONT_WAVE));		//	7 bit:	Непрерывная передача несущей (0-выкл)
	//	-//-											//	6 bit:	не используется
	CLEAR_BIT	(rf_setup_reg_value, (1<<RF_DR_LOW));	//	5 bit:	Включает низкую скорость передачи 250кбит/с. (0-выкл)
	CLEAR_BIT	(rf_setup_reg_value, (1<<PLL_LOCK));	//	4 bit:	предназначено для тестирования
	SET_BIT		(rf_setup_reg_value, (1<<RF_DR_HIGH));	//	3 bit:	Выбор скорости обмена (1 - 2Мбит/с)
	CLEAR_BIT	(rf_setup_reg_value, (1<<RF_PWR));		//	1 bit:	мощность передатчика
	CLEAR_BIT	(rf_setup_reg_value, (1<<(RF_PWR-1)));	//	0 bit:	мощность передатчика

	NRF_write_reg(RF_SETUP, rf_setup_reg_value);

	put_byte_UART2(0xF4);
	put_byte_UART2(NRF_read_reg(RF_SETUP));

//====================================================================================================================================

	//	STATUS (0x07)	//	Регистр статуса

//====================================================================================================================================

	//	OBSERVE_TX (0x08)	//	Регистр контроля передатчика

//====================================================================================================================================

	//	RPD (0x09)	//	Оценка мощности принимаемого сигнала

//====================================================================================================================================

	// Настройка регистра RX_ADDR_P0 (0x0A)		//	40-битный (5 байт) регистр, используемый для указания адреса канала 0 приёмника

	NRF_write_buf(RX_ADDR_P0, &remote_adr[0], 5);

	//	Примечание: Автоподтверждения высылаются принимающей стороной с указанием
	//				собственного адреса. Поэтому значение этого регистра должно
	//				соответствовать значению регистра TX_ADDR для корректной работы в режиме передатчика.

//====================================================================================================================================

	// Настройка регистра RX_ADDR_P1 (0x0B)		//	40-битный (5 байт) регистр, используемый для указания адреса канала 1 приёмника.
	NRF_write_buf(RX_ADDR_P1, &self_adr[0], 5);

//====================================================================================================================================

	//	RX_ADDR_P2 - RX_ADDR_P5 (0x0C-0x0F)

//====================================================================================================================================

	// Настройка регистра TX_ADDR (0x10)		//	40-битный (5 байт) регистр, используемый в режиме передатчика в качестве адреса удалённого устройства
	NRF_write_buf(TX_ADDR, &remote_adr[0], 5);

//====================================================================================================================================

	//	RX_PW_P0 - RX_PW_P5 (0x11-0x16)			//	8-битные регистры, задающие размер данных
	NRF_write_reg(RX_PW_P0, 0);					//length of incoming payload
	NRF_write_reg(RX_PW_P1, mirf_PAYLOAD);
//	NRF_write_reg(RX_PW_P2, mirf_PAYLOAD);
//	NRF_write_reg(RX_PW_P3, mirf_PAYLOAD);
//	NRF_write_reg(RX_PW_P4, mirf_PAYLOAD);
//	NRF_write_reg(RX_PW_P5, mirf_PAYLOAD);

//====================================================================================================================================
	
	// FIFO_STATUS (0x17)						//	Состояние очередей FIFO приёмника и передатчика
	
//====================================================================================================================================

	// Настройка регистра DYNPD (0x1C)			//	Разрешение использования пакетов произвольной длины
	uint8_t dynpd_reg_value = 0;

	//	-//-										//	7 bit:	не используется
	//	-//-										//	6 bit:	не используется
	CLEAR_BIT	(dynpd_reg_value, (1<<DPL_P5));		//	5 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)
	CLEAR_BIT	(dynpd_reg_value, (1<<DPL_P4));		//	4 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)
	CLEAR_BIT	(dynpd_reg_value, (1<<DPL_P3));		//	3 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)
	CLEAR_BIT	(dynpd_reg_value, (1<<DPL_P2));		//	2 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)
	SET_BIT		(dynpd_reg_value, (1<<DPL_P1));		//	1 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)
	SET_BIT		(dynpd_reg_value, (1<<DPL_P0));		//	0 bit:	запретить прием пакета произвольной длины по данному каналу (трубе)

	NRF_write_reg(DYNPD, dynpd_reg_value);

//====================================================================================================================================

	// Настройка регистра FEATURE (0x1D)			//	Регистр опций
	uint8_t feature_reg_value = 0;
	
	//	-//-											//	7 bit:	не используется
	//	-//-											//	6 bit:	не используется
	//	-//-											//	5 bit:	не используется
	//	-//-											//	4 bit:	не используется
	//	-//-											//	3 bit:	не используется
	SET_BIT		(feature_reg_value, (1<<EN_DPL));		//	2 bit:	поддержка приёма и передачи пакетов с размером поля данных произвольной длины (0-выкл)
	CLEAR_BIT	(feature_reg_value, (1<<EN_ACK_PAY));	//	1 bit:	поддержка передачи данных с пакетами подтверждения (0-выкл)
	CLEAR_BIT	(feature_reg_value, (1<<EN_DYN_ACK));	//	0 bit:	разрешает передавать пакеты, не требующие подтверждения приёма (0-выкл)

	NRF_write_reg(FEATURE, feature_reg_value);

//====================================================================================================================================

	NRF_write_reg(CONFIG, 0x0E); // Включение питания
	// Flush buffers
	NRF_cmd(FLUSH_RX);
	NRF_cmd(FLUSH_TX);

	NRF_write_reg(STATUS, NRF_read_reg(STATUS));	// сбросим флаги прерывания

}


void NRF_int_vect (void)
{
	put_byte_UART2(0xAB);

	uint8_t status_mirf;
	status_mirf = NRF_read_reg(STATUS);					//	запрашиваем статус nRF24L01


//	put_byte_UART2(NRF_read_reg(STATUS));
//	put_byte_UART2(NRF_read_reg(CONFIG));



	if(READ_BIT(status_mirf, (1<<RX_DR)))					//	если поднят флаг "получены данные"
	{
		NRF_read_buf(R_RX_PAYLOAD, &in_data_to_mirf_for_test[0], mirf_PAYLOAD);		//	считываем
		for (uint8_t i=0; i<5; i++)	{put_byte_UART2(in_data_to_mirf_for_test[i]);}		//	выведем в uart что получили
	}


NRF_cmd(FLUSH_RX);
NRF_cmd(FLUSH_TX);

NRF_write_reg(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));	// сбросим флаги прерывания


	/*
	uint8_t status_mirf;
	status_mirf = mirf_get_status();					//	запрашиваем статус nRF24L01
	uint8_t tmp_status = status_mirf | 0x8F;			//	скопируем все кроме флагов (перезапись в 1 сбрасивает флаг)

	if(READ_BIT(status_mirf,RX_DR))						//	если поднят флаг "получены данные"
	{
//		mirf_read(&MIRF_pack[0]);						//	считываем
		SET_BIT(tmp_status,RX_DR);						//	сбросим флаг успешного получения (в переменной)

//		for (uint8_t i=0; i<mirf_PAYLOAD; i++)	{put_byte(MIRF_pack[i]);}		//	выведем в uart что получили (удалить после отладки)
	}
	
	if(READ_BIT(status_mirf,TX_DS))						//	если поднят флаг "Данные успешно отправлены"
	{
		READ_BIT(tmp_status,TX_DS);						//	сбросим флаг "Данные успешно отправлены"
	}
	
	if(READ_BIT(status_mirf,MAX_RT))					//	если поднят флаг "Превышения попыток отправки"
	{
		READ_BIT(tmp_status,MAX_RT);						//	сбросим флаг "Превышения попыток отправки"
		mirf_cmd(FLUSH_TX);								//	очистим буфер отправки
	}
	
	mirf_write_register(STATUS, tmp_status);			//	сбросим флаги в NRF24L01 (запишем переменную в регистр)
	//	mirf_clear();

	 */
}

// config GPIO for CE and CSN
void GPIO_NRF_Init (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	uint8_t offset;

	// PB0 - NRF_CS
	offset = NRF_CE_pin * 4;							//	0 * 4 = 16
	GPIOB->CRL &= ~( GPIO_BITS_MASK << offset );	//	стереть 4 бита
	GPIOB->CRL |= ( OUTPUT_PUSH_PULL << offset );	//	записать 4 бита
	//GPIOB->BSRR = ( 1 << NRF_CE );				//	установка линии в 1
	//GPIOB->BRR = ( 1 << NRF_CE );					//	установка линии в 0


	// PB10 - NRF_CS								//	выбрать чип
	offset = ( NRF_CS_pin - 8 ) * 4;					//	( 10 - 8 ) * 4 = 28
	GPIOB->CRH &= ~( GPIO_BITS_MASK << offset );	//	стереть 4 бита
	GPIOB->CRH |= ( OUTPUT_PUSH_PULL << offset );	//	записать 4 бита
	//GPIOB->BSRR = ( 1 << NRF_CSN );				//	установка линии в 1
	//GPIOB->BRR = ( 1 << NRF_CSN );				//	установка линии в 0
}








/////////////////////////////////////////////////////////////////






//Similar to the previous write, clears the interrupt flags
void NRF_write (uint8_t *data)
{
	//write data
	NRF_CSN(LOW);		//	низкий уровень на CSN запускает общение с чипом по SPI
	SPI1_put_byte( W_TX_PAYLOAD );
	for (uint8_t i = 0; i < mirf_PAYLOAD; i++)   {SPI1_put_byte(data[i]);}
	NRF_CSN(HIGH);

	//start transmission
	NRF_CE(HIGH);;
	delay_us(15);
	NRF_CE(LOW);

    //Max retries exceeded
//   if (READ_BIT(status_reg, MAX_RT))	{NRF_cmd(FLUSH_TX);}	//Only going to be 1 packet int the FIFO at a time using this method, so just flush

//	NRF_write_reg(STATUS, (1<<TX_DS));	// сбросим флаг успешной передачи
}


void NRF_INIT_TEST(void)
{
	SPI1_Init();		//	для общения с чипом нужен SPI
	GPIO_NRF_Init();	//	инициализация остальных пинов к которым подключен модуль
	EXTI_Init();		//	инициализация внешнего прерывания, для реакции на прерывание от модуля

	uint8_t chan = 3; // Номер радио-канала (в диапазоне 0 - 125)

#ifdef MIRF_Master
	uint8_t self_adr[5] = 	{0xC7, 0xC7, 0xC7, 0xC7, 0xC7};
	uint8_t remote_adr[5] = {0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
#else
	uint8_t self_adr[5] = 	{0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	uint8_t remote_adr[5] = {0xC7, 0xC7, 0xC7, 0xC7, 0xC7};
#endif

	NRF_CE(LOW);					//	CE: Chip Enable. Зависит от режима работы. Если чип сконфигурен как приемник, то высокий (HIGH) уровень на CE позволяет чипу мониторить среду и получать пакеты. Низкий (LOW) уровень переводит чип в Standby-I и такая возможность становится уже недоступна. Если чип настроен на передачу, CE всегда держится на низком уровне. В этом случае для передачи данных нужно положить их в очередь FIFO и дернуть CE минимум на 10мкс (LOW->HIGH, 10мкс, HIGH->LOW).
	NRF_CSN(HIGH);					//	поднимаем линию (общение окончено) //	активный уровень - низкий (после того как пообщались - переводим в высокий) (при инициалисзации обязательно к +)


	NRF_write_reg(EN_AA, 							(1 << ENAA_P1)); 	// включение автоподтверждения только по каналу 1
	NRF_write_reg(EN_RXADDR, 	(1 << ERX_P0) | 	(1 << ERX_P1)); 	// включение каналов 0 и 1
	NRF_write_reg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); 					// выбор длины адреса 5 байт
	NRF_write_reg(SETUP_RETR, SETUP_RETR_DELAY_250MKS | SETUP_RETR_UP_TO_3_RETRANSMIT);
	NRF_write_reg(RF_CH, chan); 										// Выбор частотного канала
	NRF_write_reg(RF_SETUP, RF_SETUP_1MBPS | RF_SETUP_0DBM); 			// выбор скорости 1 Мбит/с и мощности 0dBm

	NRF_write_buf(RX_ADDR_P0, &remote_adr[0], 5); // Подтверждения приходят на канал 0
	NRF_write_buf(TX_ADDR, &remote_adr[0], 5);
	NRF_write_buf(RX_ADDR_P1, &self_adr[0], 5);

	NRF_write_reg(RX_PW_P0, 0);
	NRF_write_reg(RX_PW_P1, 32);
	NRF_write_reg(DYNPD, (1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
	NRF_write_reg(FEATURE, 0x04); // разрешение произвольной длины пакета данных

//	NRF_write_reg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX)); // Включение питания
	NRF_write_reg(CONFIG, 0x0E); // Включение питания

	// Flush buffers
	NRF_cmd(FLUSH_RX);
	NRF_cmd(FLUSH_TX);

	NRF_write_reg(STATUS, NRF_read_reg(STATUS));	// сбросим флаги прерывания
}

//#include "../defines/defines_global.h"
//#include <util/delay.h>

#include "mirf.h"
#include "nRF24L01.h"
#include "spi_1.h"
#include "spi_2.h"
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
		case HIGH:	{/*delay_us(1);*/	GPIOB->BSRR = ( 1 << NRF_CE_pin ); 				}	break;	//	поднимаем линию
		case LOW:	{				GPIOB->BRR = ( 1 << NRF_CE_pin );	/*delay_us(1);*/}	break;	//	опускаем лини
	}
}

void NRF_CSN (uint8_t value)
{
	switch (value)
	{
		case HIGH:	{/*delay_us(1); */	GPIOB->BSRR = ( 1 << NRF_CS_pin ); 				}	break;	//	поднимаем линию
		case LOW:	{ 				GPIOB->BRR = ( 1 << NRF_CS_pin );	/*delay_us(1);*/}	break;	//	опускаем лини
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
	SPI2_put_byte(cmd);
	NRF_CSN(HIGH);
}

//write many registers
void NRF_write_buf(uint8_t cmd, uint8_t *buf, uint8_t cnt)
{
	NRF_CSN(LOW);
	SPI2_put_byte(W_REGISTER | (REGISTER_MASK & cmd));
	for (uint8_t i = 0; i < cnt; i++)	{SPI2_put_byte(buf[i]);}
	NRF_CSN(HIGH);
}

//read many registers
void NRF_read_buf(uint8_t reg, uint8_t *buf, uint8_t cnt)
{
    NRF_CSN(LOW);
    SPI2_put_byte(R_REGISTER | (REGISTER_MASK & reg));
	for (uint8_t i = 0; i < cnt; i++)    {buf[i] = SPI2_put_byte(0xFF);}
    NRF_CSN(HIGH);
}

//write one register
void NRF_write_reg (uint8_t adr, uint8_t data)
{
	NRF_CSN(LOW);											//	опускаем линию (начинается общение)
	SPI2_put_byte(W_REGISTER | (REGISTER_MASK & adr));		//	отправляем команду
	SPI2_put_byte(data);									//	отправляем данные и сохраняем ответ
	NRF_CSN(HIGH);											//	поднимаем линию (общение окончено)
}

//read one register
uint8_t NRF_read_reg (uint8_t adr)
{
	uint8_t rx_byte;										//	для ответа
	NRF_CSN(LOW);											//	опускаем линию (начинается общение)
						 rx_byte = SPI2_put_byte(adr);		//	отправляем адрес регистра
	if (adr != 0xFF)	{rx_byte = SPI2_put_byte(0xFF);}	//	если ответ в следующем байте
	NRF_CSN(HIGH);											//	поднимаем линию (общение окончено)
	return rx_byte;
}

//initialize mirf



void NRF_int_vect (void)
{
	put_byte_UART2(0xAB);

	uint8_t status_mirf;
	status_mirf = NRF_read_reg(STATUS);					//	запрашиваем статус nRF24L01

	if(READ_BIT(status_mirf, (1<<RX_DR)))					//	если поднят флаг "получены данные"
	{
		NRF_read_buf(R_RX_PAYLOAD, &in_data_to_mirf_for_test[0], mirf_PAYLOAD);		//	считываем
		for (uint8_t i=0; i<5; i++)	{put_byte_UART2(in_data_to_mirf_for_test[i]);}		//	выведем в uart что получили
	}

NRF_cmd(FLUSH_RX);
NRF_cmd(FLUSH_TX);

NRF_write_reg(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));	// сбросим флаги прерывания

}

// config GPIO for CE and CSN
void GPIO_NRF_Init (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	uint8_t offset;

	// PB0 - NRF_CE (B2)
	offset = NRF_CE_pin * 4;							//	0 * 4 = 16
	GPIOB->CRL &= ~( GPIO_BITS_MASK << offset );	//	стереть 4 бита
	GPIOB->CRL |= ( OUTPUT_PUSH_PULL << offset );	//	записать 4 бита
	//GPIOB->BSRR = ( 1 << NRF_CE );				//	установка линии в 1
	//GPIOB->BRR = ( 1 << NRF_CE );					//	установка линии в 0


	// PB10 - NRF_CS (B1)								//	выбрать чип
	offset = NRF_CS_pin * 4;					//	( 10 - 8 ) * 4 = 28
	GPIOB->CRL &= ~( GPIO_BITS_MASK << offset );	//	стереть 4 бита
	GPIOB->CRL |= ( OUTPUT_PUSH_PULL << offset );	//	записать 4 бита
	//GPIOB->BSRR = ( 1 << NRF_CSN );				//	установка линии в 1
	//GPIOB->BRR = ( 1 << NRF_CSN );				//	установка линии в 0
}



/////////////////////////////////////////////////////////////////




//Similar to the previous write, clears the interrupt flags
void NRF_write (uint8_t *data)
{
	//write data
	NRF_CSN(LOW);		//	низкий уровень на CSN запускает общение с чипом по SPI
	SPI2_put_byte( W_TX_PAYLOAD );
	for (uint8_t i = 0; i < mirf_PAYLOAD; i++)   {SPI2_put_byte(data[i]);}
	NRF_CSN(HIGH);

	//start transmission
	NRF_CE(HIGH);;
	delay_us(15);
	NRF_CE(LOW);

    //Max retries exceeded
//   if (READ_BIT(status_reg, MAX_RT))	{NRF_cmd(FLUSH_TX);}	//Only going to be 1 packet int the FIFO at a time using this method, so just flush

//	NRF_write_reg(STATUS, (1<<TX_DS));	// сбросим флаг успешной передачи
}


void NRF_Init(void)
{
	SPI2_Init();		//	для общения с чипом нужен SPI
	GPIO_NRF_Init();	//	инициализация остальных пинов к которым подключен модуль
//	EXTI_Init();		//	инициализация внешнего прерывания, для реакции на прерывание от модуля

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

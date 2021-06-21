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


void MIRF_SET_TX(void)
{
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	SET_BIT(tmp_config,PWR_UP);							//	включим питание антены
	CLEAR_BIT(tmp_config,PRIM_RX);						//	переведем в режим передатчика
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	delay_ms(50);										//	время для выхода антены в штатный режим работы
	NRF_CE_RESET();										//	в ноль ( в этом режиме импульс (10 мкс) на CE запускает отправку из буфера FIFO)
}

void MIRF_SET_RX(void)
{
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	SET_BIT(tmp_config,PWR_UP);							//	включим питание антены
	SET_BIT(tmp_config,PRIM_RX);						//	переведем в режим приемника
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	delay_ms(15);										//	время для выхода антены в штатный режим работы
	NRF_CE_SET();										//	в режиме RX значение на CE(high) запускает прослушивание эфира
}

void MIRF_SET_PD(void)
{
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	CLEAR_BIT(tmp_config,PWR_UP);						//	выключим питание антены
	SET_BIT(tmp_config,PRIM_RX);						//	переведем в режим передатчика
	NRF_write_reg(CONFIG, tmp_config);					//	запишем новые настройки
	NRF_CE_RESET();
}

void MIRF_SET_PowerUp(void)
{
	// if not powered up then power up and wait for the radio to initialize
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	if (!(tmp_config & (1<<PWR_UP)))	{SET_BIT(tmp_config,(1<<PWR_UP));}
	NRF_write_reg(CONFIG, tmp_config);
	// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
	delay_us(5);
}



void setRetries(uint8_t delay, uint8_t count)
{
	NRF_write_reg(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}


uint8_t mirf_cmd(uint8_t cmd)
{
	uint8_t status = 0;
	NRF_CS_ON();
	status = SPI1_put_byte(cmd);
	NRF_CS_OFF();
	return status;
}

void mirf_powerUp(void)
{
	//��� ����������� ������� ����� ������ �������� �� SPI � ����� (���������� ����������)
	uint8_t cfg = NRF_read_reg(CONFIG);

	// if not powered up then power up and wait for the radio to initialize
	if (!(cfg & (1<<PWR_UP)))	{NRF_write_reg(CONFIG, cfg | (1<<PWR_UP));	delay_us(10);}


	// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
}


//initialize mirf
uint8_t mirf_init(void)
{
	SPI1_Init();		//	для общения с чипом нужен SPI
	GPIO_mirf_Init();	//	инициализация остальных пинов к которым подключен модуль
	EXTI_Init();		//	инициализация внешнего прерывания, для реакции на прерывание от модуля

	NRF_CE_RESET();		//	CE: Chip Enable. Зависит от режима работы. Если чип сконфигурен как приемник, то высокий (HIGH) уровень на CE позволяет чипу мониторить среду и получать пакеты. Низкий (LOW) уровень переводит чип в Standby-I и такая возможность становится уже недоступна. Если чип настроен на передачу, CE всегда держится на низком уровне. В этом случае для передачи данных нужно положить их в очередь FIFO и дернуть CE минимум на 10мкс (LOW->HIGH, 10мкс, HIGH->LOW).
	NRF_CS_OFF();		//	поднимаем линию (общение окончено) //	активный уровень - низкий (после того как пообщались - переводим в высокий) (при инициалисзации обязательно к +)

	NRF_write_reg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX));	    // Reset NRF_CONFIG and enable 16-bit CRC. (включить 16-и битную контрольную сумму)

	delay_ms(50);//	задержка, чтобы чип запустился

	setRetries(150, 2);												//	настроим паузу и количество повторных попыток
	NRF_write_reg(RF_SETUP, (1<<RF_DR_HIGH));						//	установим скорость передачи данных (max)
	NRF_write_reg(FEATURE, 0x04);									//	запретить: передавать пакеты не требующие ACK; передавать данные вместе с ACK пакетом; поле данных разной длины
	NRF_write_reg(DYNPD, (1 << DPL_P0) | (1 << DPL_P1));			//	запретим прием пакетов произвольной длины по всем каналам

	NRF_write_reg(RF_CH, mirf_CH);	//	номер радиоканала от 0 до 125

	//	8-битные регистры, задающие размер данных, принимаемых по каналам (трубам), соответственно 0-5
	NRF_write_reg(RX_PW_P0, mirf_PAYLOAD);		//length of incoming payload
	NRF_write_reg(RX_PW_P1, mirf_PAYLOAD);
	NRF_write_reg(RX_PW_P2, mirf_PAYLOAD);
	NRF_write_reg(RX_PW_P3, mirf_PAYLOAD);
	NRF_write_reg(RX_PW_P4, mirf_PAYLOAD);
	NRF_write_reg(RX_PW_P5, mirf_PAYLOAD);
	
	NRF_write_reg(EN_AA, 0);		//	(по умолчанию на 0-й канал (трубу)
	
	//�������� �������� ������ (�����) ��������. (���� �������� ���������������� �� ����� 0 ������ ���� �������� (� ���� �������� ACK)
	mirf_write_register(EN_RXADDR, 0x00);	//	��� ������ ��� ��������
	#if mirf_ENABLED_P0 == 1	
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P0));
	#endif
	#if mirf_ENABLED_P1 == 1
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P1));
	#endif
	#if mirf_ENABLED_P2 == 1
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P2));
	#endif
	#if mirf_ENABLED_P3 == 1
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P3));
	#endif
	#if mirf_ENABLED_P4 == 1
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P4));
	#endif
	#if mirf_ENABLED_P5 == 1		
		NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P5));
	#endif

	NRF_write_reg(SETUP_AW, adr_3_bytes);	//	����������� ����� ������ (������ 5 ����)

	// Flush buffers
	mirf_cmd(FLUSH_RX);
	mirf_cmd(FLUSH_TX);
	
	NRF_write_reg(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));		// сбросим флаги прерывания
	
	NRF_CS_OFF();	//	общение с чипом закончилось
	
}


//set rx address (������������� ����������� ����� ��� ������ �� ������ (�����))
void mirf_set_rxaddr(uint8_t channel, uint8_t *addr)
{
	NRF_CE_RESET();
    switch(channel)
    {
    	case 0:		{mirf_write_registers(RX_ADDR_P0, addr, 3);		break;}
		case 1:		{mirf_write_registers(RX_ADDR_P1, addr, 3);	    break;}
    	case 2:		{mirf_write_registers(RX_ADDR_P2, addr, 3);		break;}
    	case 3:		{mirf_write_registers(RX_ADDR_P3, addr, 3);		break;}
    	case 4:		{mirf_write_registers(RX_ADDR_P4, addr, 3);		break;}
    	case 5:		{mirf_write_registers(RX_ADDR_P5, addr, 3);		break;}
    }
    NRF_CE_SET();
}

//set tx address	(����� ���������� ����������. �� ��������� E7E7E7E7E7)
void mirf_set_txaddr(uint8_t *addr)
{
    mirf_write_registers(TX_ADDR, addr, 3);
}

//get status
uint8_t mirf_get_status(void)
{
	uint8_t status = 0;
	NRF_CS_ON();
	status = SPI1_put_byte(0xFF);
	NRF_CS_OFF();
	return status;
}

//read one register
uint8_t mirf_read_register(uint8_t reg)
{/*
	uint8_t result;
	MIRF_CSN_LO();
	spi_writeread(R_REGISTER | (REGISTER_MASK & reg));
	result = spi_writeread(NOP);
    MIRF_CSN_HI();
    return result;
    */
}

//read many registers
void mirf_read_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;
    MIRF_CSN(LOW);
    SPI1_put_byte(R_REGISTER | (REGISTER_MASK & reg));
	for (i = 0; i < len; i++)    {value[i] = SPI1_put_byte(0xFF);}
    MIRF_CSN(HIGH);
}

//write one register
uint8_t mirf_write_register(uint8_t reg, uint8_t value)
{
	uint8_t status = 0;
	NRF_CS_ON();

	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & reg));
//	WRITE_REG(SPI1->DR, W_REGISTER | (REGISTER_MASK & reg));

	SPI1_put_byte(value);
//	WRITE_REG(SPI1->DR, value);

	NRF_CS_OFF();

	return status;	
}

//write many registers
void mirf_write_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;
	NRF_CS_ON();
	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & reg));
	for (i = 0; i < len; i++)		{SPI1_put_byte(value[i]);}
	NRF_CS_OFF();
}

//check if there is rx data
uint8_t mirf_read_ready(void)
{
    uint8_t status = mirf_get_status();
    return (status & (1<<RX_DR));
}

uint8_t mirf_write_ready(void)
{
    uint8_t status = mirf_get_status();
    return (status & (1<<TX_DS));
}

//read data
void mirf_read(uint8_t *data)
{
	volatile uint8_t i;
	NRF_CS_ON();
	SPI1_put_byte(R_RX_PAYLOAD);
    for(i = 0; i < mirf_PAYLOAD; i++)		{data[i] = SPI1_put_byte(0xFF);}
    NRF_CS_OFF();
}


void mirf_write(uint8_t *data)
{

	volatile uint8_t i;

	NRF_CE_RESET();		//	плюс запустит отправку. Поэтому прижмем на всякий случай

	//write data
	NRF_CS_ON();		//	низкий уровень на CSN запускает общение с чипом по SPI
	SPI1_put_byte( W_TX_PAYLOAD );
	for (i = 0; i < mirf_PAYLOAD; i++)   {SPI1_put_byte(data[i]);}
	NRF_CS_OFF();

	//start transmission
	NRF_CE_SET();
	delay_us(15);
	NRF_CE_RESET();

/*	// проверим успешна ли отпрака (пришло ли подтверждение на канал 0)		 (нафиг она нужна? - он же зависнет если никто не примет. Разве что если по таймеру чистить mirf)
	while( !(mirf_get_status() & (1<<TX_DS)) )
	{
//		sei();
		_delay_us(1);
	}*/

	mirf_write_register(STATUS, (1<<TX_DS));	// сбросим флаг успешной передачи

}

void mirf_clear(void)
{
	// Flush buffers
	mirf_cmd(FLUSH_RX);
	mirf_cmd(FLUSH_TX);
		
	mirf_write_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));		// ������� ����� ����������
/*	
	uint8_t tmp = 0;
	tmp = mirf_get_status();
	//tmp = tmp & ((1<<MAX_RT)|(1<<RX_DR)|(1<<TX_DS));
	mirf_write_register(STATUS, tmp);
	*/
}

void mirf_rx_clear(void)
{
	uint8_t ttt;
	MIRF_CSN_LO();
	spi_writeread(FLUSH_RX);
	MIRF_CSN_HI();
	ttt = mirf_get_status();
	mirf_write_register(STATUS,ttt);
}




void mirf_int_vect (void)
{

	uint8_t status_mirf;
	status_mirf = mirf_get_status();					//	запрашиваем статус nRF24L01

	if(READ_BIT(status_mirf,RX_DR))					//	если поднят флаг "получены данные"
	{
		mirf_read(&in_data_to_mirf_for_test[0]);		//	считываем
		for (uint8_t i=0; i<mirf_PAYLOAD; i++)	{put_byte_UART2(in_data_to_mirf_for_test[i]);}		//	выведем в uart что получили
	}


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


void GPIO_mirf_Init (void)
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


uint8_t NRF_write_reg (uint8_t adr, uint8_t data)
{
	uint8_t rx_byte;					//	для ответа
	adr |= W_REGISTER;					//	адрес занимает меньше восьми бит, поэтому на байт наклабывается маска с указанием чтения или записи
	NRF_CSN(LOW);						//	опускаем линию (начинается общение)
				SPI1_put_byte(adr);		//	отправляем команду
	rx_byte = 	SPI1_put_byte(data);	//	отправляем данные и сохраняем ответ
	NRF_CSN(HIGH);						//	поднимаем линию (общение окончено)
	return rx_byte;
}

uint8_t NRF_read_reg (uint8_t adr)
{
	uint8_t rx_byte;										//	для ответа
	NRF_CSN(LOW);											//	опускаем линию (начинается общение)
						 rx_byte = SPI1_put_byte(adr);		//	отправляем адрес регистра
	if (adr != 0xFF)	{rx_byte = SPI1_put_byte(0xFF);}	//	если ответ в следующем байте
	NRF_CSN(HIGH);											//	поднимаем линию (общение окончено)
	return rx_byte;
}















/////////////////////////////////////////////////////////////////



void NRF_Init(void)
{

	SPI1_Init();		//	для общения с чипом нужен SPI
	GPIO_mirf_Init();	//	инициализация остальных пинов к которым подключен модуль
	EXTI_Init();		//	инициализация внешнего прерывания, для реакции на прерывание от модуля

	NRF_CE(LOW);
	NRF_CSN(HIGH);

	delay_us(100);



    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    setRetries(5, 15);

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    setDataRate(RF24_1MBPS);

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
    toggle_features();
    NRF_write_reg(FEATURE, 0);
    NRF_write_reg(DYNPD, 0);
//    dynamic_payloads_enabled = false;
//    ack_payloads_enabled = false;

    // Reset current status
    // Notice reset and flush is the last thing we do
    NRF_write_reg(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(76);

    // Flush buffers
    flush_rx();
    flush_tx();

    // Clear CONFIG register, Enable PTX, Power Up & 16-bit CRC
    // Do not write CE high so radio will remain in standby I mode
    // PTX should use only 22uA of power
//    write_register(NRF_CONFIG, (_BV(EN_CRC) | _BV(CRCO)) );
    NRF_write_reg(CONFIG, (1 << EN_CRC) | (1 << CRCO) );

    MIRF_SET_PowerUp();

}




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
		case HIGH:	{ GPIOB->BSRR = ( 1 << NRF_CS_pin ); }	break;	//	поднимаем линию
		case LOW:	{ GPIOB->BRR = ( 1 << NRF_CS_pin );	}	break;	//	опускаем лини
	}
}



uint8_t setDataRate(uint8_t speed)
{
	uint8_t result = 0;
    uint8_t setup = NRF_read_reg(RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default
    CLEAR_BIT (setup,RF_DR_LOW);
    CLEAR_BIT (setup,RF_DR_HIGH);

	switch (speed)
	{
		case RF24_1MBPS:	{CLEAR_BIT	(setup,RF_DR_LOW);		CLEAR_BIT	(setup,RF_DR_HIGH);}	break;
		case RF24_2MBPS:	{CLEAR_BIT	(setup,RF_DR_LOW);		SET_BIT		(setup,RF_DR_HIGH);}	break;
		case RF24_250KBPS:	{SET_BIT 	(setup,RF_DR_LOW);		CLEAR_BIT	(setup,RF_DR_HIGH);}	break;
	}

	NRF_write_reg(RF_SETUP, setup);

    // Verify our result
    if (NRF_read_reg(RF_SETUP) == setup) {result = 1;}

    return result;
}


void toggle_features(void)
{
    NRF_CSN(LOW);
    SPI1_put_byte(ACTIVATE);
    SPI1_put_byte(0x73);
    NRF_CSN(HIGH);
}


void setChannel(uint8_t channel)
{
    mirf_write_register(RF_CH, 87+(0x0F*2));	//	номер радиоканала от 0 до 125
    /*
    Регистр задаёт номер радиоканала - частоту несущей с шагом 1Мгц. Радиочастота несущей
    вычисляется по формуле 2400 + RF_CH МГц.
    Допустимые значения от 0 до 125. При обмене на скорости 2Мбит/с, частота должна отличатся от
    частоты используемой другими устройствами минимум на 2 МГц.
    */
}


uint8_t flush_rx(void)
{
    return SPI1_put_byte(FLUSH_RX);
}

/****************************************************************************/

uint8_t flush_tx(void)
{
    return SPI1_put_byte(FLUSH_TX);
}




void setPALevel(uint8_t level)
{

    uint8_t setup = NRF_read_red(RF_SETUP) & 0xF8;

    switch (level)
    {
		case min_PA:			{SET_BIT	(setup, (1 << (RF_PWR+1)));		SET_BIT		(setup, (1 << RF_PWR));} break;	//	low
		case low_PA:			{SET_BIT	(setup, (1 << (RF_PWR+1)));		CLEAR_BIT	(setup, (1 << RF_PWR));} break;	//	middle
		case high_PA:			{CLEAR_BIT	(setup, (1 << (RF_PWR+1)));		SET_BIT		(setup, (1 << RF_PWR));} break;	//	high
		case max_PA:			{CLEAR_BIT	(setup, (1 << (RF_PWR+1)));		CLEAR_BIT	(setup, (1 << RF_PWR));} break; //	very_high
		default:				{SET_BIT	(setup, (1 << (RF_PWR+1)));		SET_BIT		(setup, (1 << RF_PWR));} break;	//	low
    }

    NRF_write_reg(RF_SETUP, setup);
/*
    if (level > 3) {                        // If invalid level, go to max PA
        level = (RF24_PA_MAX << 1) + lnaEnable;        // +1 to support the SI24R1 chip extra bit
    } else {
        level = (level << 1) + lnaEnable;            // Else set level as requested
    }

    write_register(RF_SETUP, setup |= level);    // Write it to the chip
    */
}

void openWritingPipe( uint8_t* address)
{
	uint8_t adr_len = 3;
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
	NRF_write_registers(RX_ADDR_P0, address, adr_len);
	NRF_write_registers(TX_ADDR, address, adr_len);

    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
	NRF_write_reg(RX_PW_P0, mirf_PAYLOAD);
}





void openReadingPipe(uint8_t pipe, uint8_t* address)
{

	switch(pipe)
	{
		case 0: {NRF_write_registers(RX_ADDR_P0, &address[0], 3);}	break;
		case 1: {NRF_write_registers(RX_ADDR_P1, &address[0], 3);}	break;
		case 2: {NRF_write_registers(RX_ADDR_P2, &address[0], 3);}	break;
		case 3: {NRF_write_registers(RX_ADDR_P3, &address[0], 3);}	break;
		case 4: {NRF_write_registers(RX_ADDR_P4, &address[0], 3);}	break;
		case 5: {NRF_write_registers(RX_ADDR_P5, &address[0], 3);}	break;
	}




	/*
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, address, addr_width);
    }
    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
        } else {
            write_register(pgm_read_byte(&child_pipe[child]), address, 1);
        }
        write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

    }
    */

    /*
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, &address, addr_width);
    }

    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), addr_width);
        } else {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);
        }

        write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
    */
}


void NRF_write_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;
	NRF_CSN(LOW);
	SPI1_put_byte(W_REGISTER | (REGISTER_MASK & reg));
	for (i = 0; i < len; i++)		{SPI1_put_byte(value[i]);}
	NRF_CSN(HIGH);
}




void startListening(void)
{
    #if !defined(RF24_TINY) && !defined(LITTLEWIRE)
	MIRF_SET_PowerUp();
    #endif
    /* Notes Once ready for next release
     * 1. Can update stopListening() to use config_reg var and ack_payloads_enabled var instead of SPI rx/tx
     * 2. Update txDelay defaults: 240 for 2MBPS, 280 for 1MBPS, 505 for 250KBPS per initial testing
     * 3. Allows time for slower devices to update with the faster startListening() function prior to updating stopListening() & adjusting txDelay
     */
    uint8_t config_reg;
    uint8_t satus_reg;

    config_reg = NRF_read_reg(CONFIG);
    satus_reg = NRF_read_reg(STATUS);

    SET_BIT(config_reg, PRIM_RX);

    SET_BIT(satus_reg, RX_DR);
    SET_BIT(satus_reg, TX_DS);
    SET_BIT(satus_reg, MAX_RT);

    NRF_write_reg(CONFIG, config_reg);
    NRF_write_reg(STATUS, satus_reg);

    flush_tx();

    NRF_CE(HIGH);
}

void stopListening(void)
{
	NRF_CE(LOW);
	delay_us(20);

	uint8_t feature_reg = NRF_read_reg(FEATURE);
	if(READ_BIT(feature_reg, EN_ACK_PAY))	{delay_ms(50); flush_tx();}

	uint8_t config_reg = NRF_read_reg(CONFIG);
	CLEAR_BIT(config_reg, PRIM_RX);
	NRF_write_reg(CONFIG, config_reg);

	NRF_write_reg(EN_RXADDR, NRF_read_reg(EN_RXADDR) | (1<<ERX_P0));

}


//Similar to the previous write, clears the interrupt flags
void write (uint8_t *data)
{
	NRF_CE(LOW);		//	плюс запустит отправку. Поэтому прижмем на всякий случай

	//write data
	NRF_CSN(LOW);		//	низкий уровень на CSN запускает общение с чипом по SPI
	SPI1_put_byte( W_TX_PAYLOAD );
	for (uint8_t i = 0; i < mirf_PAYLOAD; i++)   {SPI1_put_byte(data[i]);}
	NRF_CSN(HIGH);

	uint8_t status_reg = NRF_write_reg(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

	//start transmission
	NRF_CE(HIGH);;
	delay_us(15);
	NRF_CE(LOW);

    //Max retries exceeded
    if (READ_BIT(status_reg, MAX_RT))	{flush_tx();}	//Only going to be 1 packet int the FIFO at a time using this method, so just flush

//	NRF_write_reg(STATUS, (1<<TX_DS));	// сбросим флаг успешной передачи
}


void read(uint8_t* buf, uint8_t len)
{

    // Fetch the payload
    read_payload(buf, len);

    //Clear the two possible interrupt flags with one command
    NRF_write_reg(STATUS, (1 << RX_DR) | (1 << MAX_RT) | (1 << TX_DS));

}




uint8_t read_payload(uint8_t* buf, uint8_t data_len)
{
    NRF_CSN(LOW);
    SPI1_put_byte(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < mirf_PAYLOAD; i++)		{buf[i] = SPI1_put_byte(0xFF);}
    NRF_CSN(HIGH);

    NRF_write_reg(STATUS, (1<<RX_DR));		//	сбросим флаг успешного получения
}


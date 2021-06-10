#include "../defines/defines_global.h"
#include <util/delay.h>
#include "mirf.h"
#include "nRF24L01.h"
#include "../spi/spi.h"

#include "../uart/uart.h"	//	для отладки
#include <avr/interrupt.h>

//extern			uint8_t		MIRF_pack[MAX_PACK_LENGTH];

void MIRF_SET_TX(void)
{
	cli();
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	sbit(tmp_config,PWR_UP);							//	включим питание антены
	cbit(tmp_config,PRIM_RX);							//	переведем в режим передатчика
	mirf_write_register(CONFIG, tmp_config);			//	запишем новые настройки
	MIRF_CE_LO();										//	в ноль ( в этом режиме импульс (10 мкс) на CE запускает отправку из буфера FIFO)
	_delay_ms(50);										//	время для выхода антены в штатный режим работы
	sei();
}

void MIRF_SET_RX(void)
{
	cli();
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	sbit(tmp_config,PWR_UP);							//	включим питание антены
	sbit(tmp_config,PRIM_RX);							//	переведем в режим приемника
	mirf_write_register(CONFIG, tmp_config);			//	запишем новые настройки
	MIRF_CE_HI();										//	в режиме RX значение на CE(high) запускает прослушивание эфира
	_delay_ms(50);										//	время для выхода антены в штатный режим работы
	sei();
}

void MIRF_SET_PD(void)
{
	uint8_t tmp_config = mirf_read_register(CONFIG);	//	получим текущие настройки из чипа
	cbit(tmp_config,PWR_UP);							//	выключим питание антены
	sbit(tmp_config,PRIM_RX);							//	переведем в режим передатчика
	mirf_write_register(CONFIG, tmp_config);			//	запишем новые настройки
	MIRF_CE_LO();
}

void setRetries(uint8_t delay, uint8_t count)
{
	mirf_write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

uint8_t mirf_cmd(uint8_t cmd)
{
	uint8_t status = 0;
	MIRF_CSN_LO();
	status = spi_writeread(cmd);
	MIRF_CSN_HI();
	return status;
}

void mirf_powerUp(void)
{
	//без включенного питания можно только общаться по SPI с чипом (радиообмен невозможен)
	uint8_t cfg = mirf_read_register(CONFIG);

	// if not powered up then power up and wait for the radio to initialize
	if (!(cfg & (1<<PWR_UP)))	{mirf_write_register(CONFIG, cfg | (1<<PWR_UP));	_delay_us(10);}


	// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
}


//initialize mirf
uint8_t mirf_init(void)
{
	//	Порты
	//	Внешнее рерывание
	//	SPI
		
	uint8_t setup = 0;
	    
	MIRF_CE_LO();	//	CE: Chip Enable. Зависит от режима работы. Если чип сконфигурен как приемник, то высокий (HIGH) уровень на CE позволяет чипу мониторить среду и получать пакеты. Низкий (LOW) уровень переводит чип в Standby-I и такая возможность становится уже недоступна. Если чип настроен на передачу, CE всегда держится на низком уровне. В этом случае для передачи данных нужно положить их в очередь FIFO и дернуть CE минимум на 10мкс (LOW->HIGH, 10мкс, HIGH->LOW).
	MIRF_CSN_HI();	//	активный уровень - низкий (после того как пообщались - переводим в высокий) (при инициалисзации обязательно к +)

	mirf_write_register(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX));	    // Reset NRF_CONFIG and enable 16-bit CRC. (включить 16-и битную контрольную сумму)
    
//	setRetries(150, 2);	//	настроим паузу и количество повторных попыток
		
	mirf_write_register(RF_SETUP, (1<<RF_DR_HIGH));			//	установим скорость передачи данных (max)
	mirf_write_register(FEATURE, 0x04);																		//	запретить: передавать пакеты не требующие ACK; передавать данные вместе с ACK пакетом; поле данных разной длины
	mirf_write_register(DYNPD, (1 << DPL_P0) | (1 << DPL_P1));												//	запретим прием пакетов произвольной длины по всем каналам

	mirf_write_register(RF_CH, mirf_CH);	//	номер радиоканала от 0 до 125
	
	//	8-битные регистры, задающие размер данных, принимаемых по каналам (трубам), соответственно 0-5
	mirf_write_register(RX_PW_P0, mirf_PAYLOAD);		//length of incoming payload
	mirf_write_register(RX_PW_P1, mirf_PAYLOAD);
	mirf_write_register(RX_PW_P2, mirf_PAYLOAD);
	mirf_write_register(RX_PW_P3, mirf_PAYLOAD);
	mirf_write_register(RX_PW_P4, mirf_PAYLOAD);
	mirf_write_register(RX_PW_P5, mirf_PAYLOAD);
	
	mirf_write_register(EN_AA, 0x00);	// указывает по каким трубам отправлять автоподтвержедние
	
	//Выбирает активные каналы (трубы) приёмника. (если включено автопотверждение то канал 0 должен быть активный (в него приходят ACK)
	mirf_write_register(EN_RXADDR, 0x00);	//	для начало все отключим
	#if mirf_ENABLED_P0 == 1	
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P0));
	#endif
	#if mirf_ENABLED_P1 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P1));
	#endif
	#if mirf_ENABLED_P2 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P2));
	#endif
	#if mirf_ENABLED_P3 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P3));
	#endif
	#if mirf_ENABLED_P4 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P4));
	#endif
	#if mirf_ENABLED_P5 == 1		
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P5));
	#endif


	mirf_write_register(SETUP_AW, adr_3_bytes);	//	настраивает длину адреса (длиной 5 байт)

	// Flush buffers
	mirf_cmd(FLUSH_RX);
	mirf_cmd(FLUSH_TX);
	
	mirf_write_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));		// сбросим флаги прерывания
	
	MIRF_CSN_HI();	//	конец общения с чипом
	
	// if setup is 0 or ff then there was no response from module
	return (setup != 0 && setup != 0xff);
}


//set rx address (устанавливает собственный адрес для одного из канала (трубы))
void mirf_set_rxaddr(uint8_t channel, uint8_t *addr)
{
    MIRF_CE_LO();
    switch(channel)
    {
    	case 0:		{mirf_write_registers(RX_ADDR_P0, addr, 3);		break;}
		case 1:		{mirf_write_registers(RX_ADDR_P1, addr, 3);	    break;}
    	case 2:		{mirf_write_registers(RX_ADDR_P2, addr, 3);		break;}
    	case 3:		{mirf_write_registers(RX_ADDR_P3, addr, 3);		break;}
    	case 4:		{mirf_write_registers(RX_ADDR_P4, addr, 3);		break;}
    	case 5:		{mirf_write_registers(RX_ADDR_P5, addr, 3);		break;}
    }
    MIRF_CE_HI();
}

//set tx address	(адрес удаленного устройства. по умолчанию E7E7E7E7E7)
void mirf_set_txaddr(uint8_t *addr)
{
    mirf_write_registers(TX_ADDR, addr, 3);
}

//get status
uint8_t mirf_get_status(void)
{
	uint8_t status = 0;
	MIRF_CSN_LO();
	status = spi_writeread(NOP);
	MIRF_CSN_HI();
	return status;
}

//read one register
uint8_t mirf_read_register(uint8_t reg)
{
	uint8_t result;
	MIRF_CSN_LO();
	spi_writeread(R_REGISTER | (REGISTER_MASK & reg));
	result = spi_writeread(NOP);
    MIRF_CSN_HI();
    return result;
}

//read many registers
void mirf_read_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;
    MIRF_CSN_LO();
	spi_writeread(R_REGISTER | (REGISTER_MASK & reg));
	for (i = 0; i < len; i++)    {value[i] = spi_writeread(NOP);}
    MIRF_CSN_HI();
}

//write one register
uint8_t mirf_write_register(uint8_t reg, uint8_t value)
{
	uint8_t status = 0;
	MIRF_CSN_LO();
	spi_writeread(W_REGISTER | (REGISTER_MASK & reg));
	spi_writeread(value);
	MIRF_CSN_HI();
	return status;	
}

//write many registers
void mirf_write_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t i;
	MIRF_CSN_LO();
	spi_writeread(W_REGISTER | (REGISTER_MASK & reg));
	for (i = 0; i < len; i++)		{spi_writeread(value[i]);}
	MIRF_CSN_HI();
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
    MIRF_CSN_LO();
	spi_writeread(R_RX_PAYLOAD);
    for(i = 0; i < mirf_PAYLOAD; i++)		{data[i] = spi_writeread(NOP);}
    MIRF_CSN_HI();
}


void mirf_write(uint8_t *data)
{
	volatile uint8_t i;

	MIRF_CE_LO();		//	плюс запустит отправку. Поэтому прижмем на всякий случай

	//write data
	MIRF_CSN_LO();		//	низкий уровень на CSN запускает общение с чипом по SPI
	spi_writeread( W_TX_PAYLOAD );
	for (i = 0; i < mirf_PAYLOAD; i++)   {spi_writeread(data[i]);}
	MIRF_CSN_HI();
	
	//start transmission
	MIRF_CE_HI();
	_delay_us(15);
	MIRF_CE_LO();	

/*	// проверим успешна ли отпрака (пришло ли подтверждение на канал 0)		 (нафиг она нужна? - он же зависнет если никто не примет. Разве что если по таймеру чистить mirf)				
	while( !(mirf_get_status() & (1<<TX_DS)) )
	{
//		sei();
		_delay_us(1);
	}*/
}

void mirf_clear(void)
{
	// Flush buffers
	mirf_cmd(FLUSH_RX);
	mirf_cmd(FLUSH_TX);
		
	mirf_write_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));		// сбросим флаги прерывания
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
	uint8_t tmp_status = status_mirf | 0x8F;			//	скопируем все кроме флагов (перезапись в 1 сбрасивает флаг)

	if(bit_is_set(status_mirf,RX_DR))					//	если поднят флаг "получены данные"
	{
//		mirf_read(&MIRF_pack[0]);						//	считываем
		sbit(tmp_status,RX_DR);							//	сбросим флаг успешного получения (в переменной)

//		for (uint8_t i=0; i<mirf_PAYLOAD; i++)	{put_byte(MIRF_pack[i]);}		//	выведем в uart что получили (удалить после отладки)
	}
	
	if(bit_is_set(status_mirf,TX_DS))					//	если поднят флаг "Данные успешно отправлены"
	{
		sbit(tmp_status,TX_DS);							//	сбросим флаг "Данные успешно отправлены"
	}
	
	if(bit_is_set(status_mirf,MAX_RT))					//	если поднят флаг "Превышения попыток отправки"
	{
		sbit(tmp_status,MAX_RT);						//	сбросим флаг "Превышения попыток отправки"
		mirf_cmd(FLUSH_TX);								//	очистим буфер отправки
	}
	
	mirf_write_register(STATUS, tmp_status);			//	сбросим флаги в NRF24L01 (запишем переменную в регистр)
	//	mirf_clear();
}
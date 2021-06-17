#ifndef _MIRF_H_
#define _MIRF_H_ 1

#include <stdio.h>
#include "nRF24L01.h"


#define	MIRF_Master		//	это устройство - мастер. Если подчиненный, то закоментировать эту строку




#define NRF_CS		10	//	B10
#define NRF_CE		0	//	B0


#define NRF_CS_OFF() 	GPIOB->BSRR = ( 1 << NRF_CS )	//	поднимаем линию
#define NRF_CS_ON() 	GPIOB->BRR = ( 1 << NRF_CS )	//	опускаем лини

#define NRF_CE_SET() 	GPIOB->BSRR = ( 1 << NRF_CE )	//	поднимаем линию
#define NRF_CE_RESET() 	GPIOB->BRR = ( 1 << NRF_CE )	//	опускаем лини


void GPIO_mirf_Init (void);
uint8_t NRF_read_reg (uint8_t adr);
uint8_t NRF_write_reg (uint8_t adr, uint8_t data);




/*
#define mirf_PAYLOAD MAX_PACK_LENGTH_FOR_MIRF
*/
#ifndef uint16_t
#define uint8_t unsigned char
#define uint16_t unsigned int
#endif

//	���������������� ��������� ���� nRF24L01
//#define	MIRF_Master		//	��� ���������� - ������. ���� �����������, �� ��������������� ��� ������


#define mirf_PAYLOAD 16



#define mirf_CH      87+(0x0F*2)//124								//transmission channel	//	����� �����������
//#define mirf_PAYLOAD 12												//payload lenght
#define mirf_CONFIG ((1<<EN_CRC) | (1<<CRCO) | (1<<MASK_TX_DS) | (1<<MASK_RX_DR))		//	 �������� 16 ��� CRC � ���������� TX, ���������� RX (���� ��� ���������� ����� ��������� �������)

#define mirf_ACK 1			//auto ack enabled	//	�������� ��� ��������� �����������������
//#define mirf_RETR 0x00		//auto ack enabled	//	(4-� ����) ����� �������� � (4-� ����) ���������� �������� ��� �� ��������� �����������������

#define mirf_ENABLED_P0 1	//	�������� 0-� �����
#define mirf_ENABLED_P1 1	//	�������� 1-� �����	(���� ����)
#define mirf_ENABLED_P2 0
#define mirf_ENABLED_P3 0
#define mirf_ENABLED_P4 0
#define mirf_ENABLED_P5 0

/*
//	��������������� �������
#define MIRF_CSN_HI() do{sbit(NRF_CSN_PORT,NRF_CSN_PIN_NUM);}while(0)		//	���������� �����
#define MIRF_CSN_LO() do{cbit(NRF_CSN_PORT,NRF_CSN_PIN_NUM);}while(0)		//	�������� �����

#define MIRF_CE_HI() do{sbit(NRF_CE_PORT,NRF_CE_PIN_NUM);}while(0)
#define MIRF_CE_LO() do{cbit(NRF_CE_PORT,NRF_CE_PIN_NUM);}while(0)
*/

uint8_t spi_writeread (uint8_t data);

uint8_t mirf_init(void);
uint8_t mirf_get_status(void);
void mirf_set_rxaddr(uint8_t channel, uint8_t *addr);
void mirf_set_txaddr(uint8_t *addr);
uint8_t mirf_read_ready(void);
uint8_t mirf_write_ready(void);
void mirf_read(uint8_t *data);
void mirf_write(uint8_t *data);

uint8_t mirf_write_register(uint8_t reg, uint8_t value);
void mirf_write_registers(uint8_t reg, uint8_t *value, uint8_t len);
uint8_t mirf_read_register(uint8_t reg);
void mirf_read_registers(uint8_t reg, uint8_t *value, uint8_t len);

void mirf_clear(void);
void mirf_rx_clear(void);


void MIRF_SET_TX(void);
void MIRF_SET_RX(void);
void MIRF_SET_PD(void);

void mirf_int_vect (void);

#endif /* _MIRF_H_ */


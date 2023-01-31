#ifndef FM25V02
#define FM25V02

#include "main.h"

// Структура команд
#define WREN  0b00000110 // Установить защиту от записи
#define WRDI  0b00000100 // Снять защиту от записи
#define RDSR  0b00000101 // Прочитать статус регистр
#define WRSR  0b00000001 // Записать статус регистр
#define READ  0b00000011 // Чтение данных из памяти
#define FSTRD 0b00001011 // Быстрое чтение данных из памяти
#define WRITE 0b00000010 // Записать данные в память
#define SLEEP 0b10111001 // Спящий режим
#define RDID  0b10011111 // Читать ID устройства

// Биты status регистра
#define WPEN 0b10000000
#define BP1  0b00001000
#define BP0  0b00000100
#define WEL  0b00000010

#define FM25V02_CS_ON()  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET)// включение CS-пина в низкое состояние
#define FM25V02_CS_OFF() HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET)//   отключение CS-пина в высокое состояние

//Карта памяти регистров FRAM----------------------------------------------
#define	SECURITY_STATE_BYTE 0x0000
#define IBUTTON0_STATE 0x0001
#define IBUTTON0_STATE_OFF 0x00
#define IBUTTON0_STATE_ON  0x01

#define IBUTTON1_H 	0x0002
#define IBUTTON1_L 	0x0003
#define IBUTTON2_H 	0x0004
#define IBUTTON2_L 	0x0005
#define IBUTTON3_H 	0x0006
#define IBUTTON3_L 	0x0007
#define IBUTTON4_H 	0x0008
#define IBUTTON4_L 	0x0009
#define IBUTTON5_H	0x000A
#define IBUTTON5_L 	0x000B
#define IBUTTON6_H 	0x000C
#define IBUTTON6_L 	0x000D
#define IBUTTON7_H 	0x000E
#define IBUTTON7_L 	0x000F
#define IBUTTON8_H 	0x0010
#define IBUTTON8_L 	0x0011
#define IBUTTON9_H 	0x0012
#define IBUTTON9_L 	0x0013
#define IBUTTON10_H 0x0014
#define IBUTTON10_L 0x0015
//-------------------------------------------------------------------------

typedef struct{
	uint8_t byte8;
	uint8_t byte7;
	uint8_t byte6;
	uint8_t byte5;
	uint8_t byte4;
	uint8_t byte3;
	uint8_t byte2;
	uint8_t byte1;
	uint8_t byte0;
} Fm25v02IdStruct;

void fm25v02_wren (void);
void fm25v02_wrdi (void);
void fm25v02_sleep (void);
uint8_t fm25v02_rdsr (void); // Функция чтения статус регистра
Fm25v02IdStruct fm25v02_rdid (void); // Функция чтения ID устройства
void fm25v02_read (uint16_t address, uint8_t* data);
void fm25v02_write (uint16_t address, uint8_t data);
HAL_StatusTypeDef fm25v02_fast_read (uint16_t address, uint8_t* data, uint16_t Size);
uint8_t fm25v02_fast_write (uint16_t address, uint8_t* data, uint16_t Size);
















#endif

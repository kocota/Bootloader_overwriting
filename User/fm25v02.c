#include "fm25v02.h"
#include "gpio.h"

extern SPI_HandleTypeDef hspi2;
uint32_t counter = 0;

void fm25v02_wren (void)
{
	uint8_t data_out = WREN;
	FM25V02_CS_ON();
	while( HAL_SPI_Transmit(&hspi2, &data_out, 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
}

void fm25v02_wrdi (void)
{
	uint8_t data_out = WRDI;
	FM25V02_CS_ON();
	while( HAL_SPI_Transmit(&hspi2, &data_out, 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
}

void fm25v02_sleep (void)
{
	uint8_t data_out = SLEEP;
	FM25V02_CS_ON();
	while( HAL_SPI_Transmit(&hspi2, &data_out, 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
}

uint8_t fm25v02_rdsr (void)
{
	uint8_t data_out = RDSR;
	uint8_t data_in;
	FM25V02_CS_ON();

	while( HAL_SPI_Transmit(&hspi2, &data_out, 1, 10) != HAL_OK ) {}
	while( HAL_SPI_Receive(&hspi2, &data_in, 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
	return data_in;
}


Fm25v02IdStruct fm25v02_rdid (void)
{
	Fm25v02IdStruct temp_id;
	uint8_t data_out = RDID;
	uint8_t data_in[9];
	data_in[0]=0xFF;
	FM25V02_CS_ON();

	while( HAL_SPI_Transmit(&hspi2, &data_out, 1, 10) != HAL_OK ) {}
	while( HAL_SPI_Receive(&hspi2, &data_in[0], 9, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
	for(uint8_t i=0; i<9; i++)
	{
		*(&temp_id.byte8+i) = data_in[i];
	}
	return temp_id;
}

void fm25v02_read (uint16_t address, uint8_t* data)
{
	uint8_t data_out[3];
	data_out[0]= READ;
	data_out[1]= (uint8_t)((address>>8)&0x00FF);
	data_out[2]= (uint8_t)(address&0x00FF);
	FM25V02_CS_ON();

	while( HAL_SPI_Transmit(&hspi2, &data_out[0], 3, 10) != HAL_OK ) {}
	while( HAL_SPI_Receive(&hspi2, data, 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
}

void fm25v02_write (uint16_t address, uint8_t data)
{
	uint8_t data_out[4];
	data_out[0]= WREN;
	FM25V02_CS_ON();
	while( HAL_SPI_Transmit(&hspi2, &data_out[0], 1, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
	data_out[0]= WRITE;
	data_out[1]= (uint8_t)((address>>8)&0x00FF);
	data_out[2]= (uint8_t)(address&0x00FF);
	data_out[3]= data;
	FM25V02_CS_ON();
	while( HAL_SPI_Transmit(&hspi2, &data_out[0], 4, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
}

uint8_t fm25v02_fast_write (uint16_t address, uint8_t* data, uint16_t Size)
{
	for(uint8_t i=0; i<Size; i++)
	{
		fm25v02_write(address+i, *(data+i));
	}
	return 0;
}

HAL_StatusTypeDef fm25v02_fast_read (uint16_t address, uint8_t* data, uint16_t Size)
{
	uint8_t data_out[4];
	data_out[0]= FSTRD;
	data_out[1]= (uint8_t)((address>>8)&0x00FF);
	data_out[2]= (uint8_t)(address&0x00FF);
	data_out[3]= 0xFF;
	FM25V02_CS_ON();

	while( HAL_SPI_Transmit(&hspi2, &data_out[0], 4, 10) != HAL_OK ) {}
	while( HAL_SPI_Receive(&hspi2, data, Size, 10) != HAL_OK ) {}
	FM25V02_CS_OFF();
	return HAL_OK;
}



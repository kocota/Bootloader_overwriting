#include "main.h"
#include "MainTask.h"
#include "modbus.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "fm25v02.h"
#include "m95.h"


extern osThreadId M95TaskHandle;
extern osThreadId MainTaskHandle;
extern osMutexId Fm25v02MutexHandle;
extern control_register_struct control_registers;
extern bootloader_register_struct bootloader_registers;
extern change_boot_register_struct change_boot_registers;

extern volatile uint8_t modem_reset_state;


//uint16_t packet_crc;
//uint32_t calculating_packet_crc;
//uint8_t buffer_packet_data[256];
//uint32_t address_to_read_write;
//uint32_t data_to_write;
uint32_t sector_error;

FLASH_EraseInitTypeDef erase_init;



//extern SPI_HandleTypeDef hspi2;
//extern UART_HandleTypeDef huart3;

uint32_t change_boot_calculating_firmware_crc1;
uint32_t change_boot_calculating_firmware_crc2;
uint32_t change_boot_start_address;
uint32_t change_boot_end_address;
uint32_t change_boot_firmware_lenght;
uint16_t change_boot_firmeware_crc;
uint32_t change_boot_address_to_write;




/*
typedef void (*pFunction) (void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t ApplicationAddress2 = 0x08010000;

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
*/


void ThreadMainTask(void const * argument)
{

	//uint8_t temp_read_h;
	//uint8_t temp_read_l;

	osThreadSuspend(MainTaskHandle); // ждем пока не будут вычитаны регистры и не получен статус фаз А1,А2,В1,В2,С1,С2







	for(;;)
	{

		switch(control_registers.reset_control_reg) // удаленная перезагрузка контроллера
		{
			case(1):
				osMutexWait(Fm25v02MutexHandle, osWaitForever);
				fm25v02_write(2*RESET_CONTROL_REG, 0);
				fm25v02_write(2*RESET_CONTROL_REG+1, 0);
				osMutexRelease(Fm25v02MutexHandle);
				NVIC_SystemReset();
			break;

		}

		switch(bootloader_registers.working_mode_reg)
		{
			case(1):

				//NVIC_SystemReset();

			break;
		}


		switch(change_boot_registers.change_boot_write_reg)
		{
			case(1):

				change_boot_address_to_write = ((((uint32_t)(change_boot_registers.change_boot_address_to_write_3_reg))<<24)&0xFF000000) | ((((uint32_t)(change_boot_registers.change_boot_address_to_write_2_reg))<<16)&0x00FF0000) | ((((uint32_t)(change_boot_registers.change_boot_address_to_write_1_reg))<<8)&0x0000FF00) | (((uint32_t)(change_boot_registers.change_boot_address_to_write_0_reg))&0x000000FF); // адрес куда писать загрузчик

				change_boot_start_address = ((((uint32_t)(change_boot_registers.change_boot_start_address_3_reg))<<24)&0xFF000000) | ((((uint32_t)(change_boot_registers.change_boot_start_address_2_reg))<<16)&0x00FF0000) | ((((uint32_t)(change_boot_registers.change_boot_start_address_1_reg))<<8)&0x0000FF00) | (((uint32_t)(change_boot_registers.change_boot_start_address_0_reg))&0x000000FF); // стартовый адрес загрузчика

				change_boot_end_address = ((((uint32_t)(change_boot_registers.change_boot_end_address_3_reg))<<24)&0xFF000000) | ((((uint32_t)(change_boot_registers.change_boot_end_address_2_reg))<<16)&0x00FF0000) | ((((uint32_t)(change_boot_registers.change_boot_end_address_1_reg))<<8)&0x0000FF00) | (((uint32_t)(change_boot_registers.change_boot_end_address_0_reg))&0x000000FF); // конечный адрес загрузчика

				change_boot_firmware_lenght = change_boot_end_address - change_boot_start_address + 1; // длина прошивки загрузчика

				change_boot_firmeware_crc = (((change_boot_registers.change_boot_crc_low_reg)<<8)&0xFF00) | ((change_boot_registers.change_boot_crc_high_reg)&0x00FF); // контрольная сумма загрузчика

				change_boot_calculating_firmware_crc1 = CRC16((unsigned char*)change_boot_start_address, change_boot_firmware_lenght); // расчитываем контрольную сумму загрузчика

				/*
				change_boot_address_to_write = 0x08000000; // адрес куда писать загрузчик, вписать сюда

				change_boot_start_address = 0x08020000; // стартовый адрес загрузчика, вписать сюда

				change_boot_end_address = 0x0802FEEF; // конечный адрес загрузчика, вписать сюда

				change_boot_firmware_lenght = change_boot_end_address - change_boot_start_address + 1; // длина прошивки загрузчика

				change_boot_firmeware_crc = 0x1BDE; // контрольная сумма загрузчика, вписать сюда

				change_boot_calculating_firmware_crc1 = CRC16((unsigned char*)change_boot_start_address, change_boot_firmware_lenght); // расчитываем контрольную сумму загрузчика
				*/

				if( change_boot_calculating_firmware_crc1 == change_boot_firmeware_crc ) // если контрольная сумма загрузчика сходится
				{

					erase_init.TypeErase = FLASH_TYPEERASE_SECTORS; // заполняем структуру с параметрами очистки памяти
					erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
					erase_init.Sector = FLASH_SECTOR_0;
					erase_init.NbSectors = 1;
					erase_init.Banks = 1;

					HAL_FLASH_Unlock(); // разблокируем запись памяти контроллера

					while( HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK ) // выполняем очистку указанной страницы памяти
					{

					}

					HAL_FLASH_Lock(); // блокируем запись памяти контроллера

					erase_init.TypeErase = FLASH_TYPEERASE_SECTORS; // заполняем структуру с параметрами очистки памяти
					erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
					erase_init.Sector = FLASH_SECTOR_1;
					erase_init.NbSectors = 1;
					erase_init.Banks = 1;

					HAL_FLASH_Unlock(); // разблокируем запись памяти контроллера

					while( HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK ) // выполняем очистку указанной страницы памяти
					{

					}

					HAL_FLASH_Lock(); // блокируем запись памяти контроллера

					erase_init.TypeErase = FLASH_TYPEERASE_SECTORS; // заполняем структуру с параметрами очистки памяти
					erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
					erase_init.Sector = FLASH_SECTOR_2;
					erase_init.NbSectors = 1;
					erase_init.Banks = 1;

					HAL_FLASH_Unlock(); // разблокируем запись памяти контроллера

					while( HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK ) // выполняем очистку указанной страницы памяти
					{

					}

					HAL_FLASH_Lock(); // блокируем запись памяти контроллера

					erase_init.TypeErase = FLASH_TYPEERASE_SECTORS; // заполняем структуру с параметрами очистки памяти
					erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
					erase_init.Sector = FLASH_SECTOR_3;
					erase_init.NbSectors = 1;
					erase_init.Banks = 1;

					HAL_FLASH_Unlock(); // разблокируем запись памяти контроллера

					while( HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK ) // выполняем очистку указанной страницы памяти
					{

					}

					HAL_FLASH_Lock(); // блокируем запись памяти контроллера




					HAL_FLASH_Unlock(); // разблокируем запись памяти контроллера
					for(uint32_t i=0; i<change_boot_firmware_lenght; i++)
					{
						while( HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, change_boot_address_to_write+i, *( (uint32_t*)(change_boot_start_address+i) )) != HAL_OK ) // ничего не делаем пока не выполнится запись в память контроллера
						{

						}
					}
					HAL_FLASH_Lock(); // блокируем запись памяти контроллера

					change_boot_calculating_firmware_crc2 = CRC16((unsigned char*)change_boot_address_to_write, change_boot_firmware_lenght); // расчитываем контрольную сумму записанного загрузчика

					if( change_boot_calculating_firmware_crc2 == change_boot_firmeware_crc ) // если контрольная сумма записанного загрузчика сходится
					{
						osMutexWait(Fm25v02MutexHandle, osWaitForever); // записываем в регистр корректность прошивки бутлоадера
						fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG, 0x00);
						fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG+1, 0x01);
						change_boot_registers.change_boot_crc_correctness_reg = 0x0001;
						osMutexRelease(Fm25v02MutexHandle);

						osMutexWait(Fm25v02MutexHandle, osWaitForever); // обнуляем регистр записи загрузчика
						fm25v02_write(2*CHANGE_BOOT_WRITE_REG, 0x00);
						fm25v02_write(2*CHANGE_BOOT_WRITE_REG+1, 0x00);
						change_boot_registers.change_boot_write_reg = 0x0000;
						osMutexRelease(Fm25v02MutexHandle);

						NVIC_SystemReset(); // перезагружаем контроллер

					}
					else
					{
						osMutexWait(Fm25v02MutexHandle, osWaitForever); // обнуляем корректность прошивки бутлоадера
						fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG, 0x00);
						fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG+1, 0x00);
						change_boot_registers.change_boot_crc_correctness_reg = 0x0000;
						osMutexRelease(Fm25v02MutexHandle);

						osMutexWait(Fm25v02MutexHandle, osWaitForever); // обнуляем регистр записи загрузчика
						fm25v02_write(2*CHANGE_BOOT_WRITE_REG, 0x00);
						fm25v02_write(2*CHANGE_BOOT_WRITE_REG+1, 0x00);
						change_boot_registers.change_boot_write_reg = 0x0000;
						osMutexRelease(Fm25v02MutexHandle);
					}

				}
				else // если контрольная сумма загрузчика не сходится
				{

					osMutexWait(Fm25v02MutexHandle, osWaitForever); // обнуляем корректность прошивки бутлоадера
					fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG, 0x00);
					fm25v02_write(2*CHANGE_BOOT_CRC_CORRECTNESS_REG+1, 0x00);
					change_boot_registers.change_boot_crc_correctness_reg = 0x0000;
					osMutexRelease(Fm25v02MutexHandle);

					osMutexWait(Fm25v02MutexHandle, osWaitForever); // обнуляем регистр записи загрузчика
					fm25v02_write(2*CHANGE_BOOT_WRITE_REG, 0x00);
					fm25v02_write(2*CHANGE_BOOT_WRITE_REG+1, 0x00);
					change_boot_registers.change_boot_write_reg = 0x0000;
					osMutexRelease(Fm25v02MutexHandle);
				}


			break;
		}

		if( modem_reset_state == 1)
		{
			osMutexWait(Fm25v02MutexHandle, osWaitForever); // ждем освобождение мьютекса записи в память
			osThreadSuspend(M95TaskHandle);
			modem_reset_state = 0;
			//AT_QPOWD(0);
			m95_power_off();
			HAL_Delay(5000);
			NVIC_SystemReset();
		}





		osDelay(1000);
	}
}

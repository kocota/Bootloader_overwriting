#include "ReadRegistersTask.h"
#include "cmsis_os.h"
#include "modbus.h"

extern osThreadId M95TaskHandle;
extern osThreadId MainTaskHandle;


void ThreadReadRegistersTask(void const * argument)
{
	read_status_registers(); // вычитываем регистры
	read_control_registers();
	read_bootloader_registers();
	read_change_boot_registers();
	osDelay(2000); // ждем пока будет получен статус фаз А1,А2,В1,В2,С1,С2
	osThreadResume(MainTaskHandle); // запускаем основной процесс
	osThreadResume(M95TaskHandle);  // запускаем процесс модема
	osDelay(1000); //ждем 1 секунду


	for(;;)
	{

		read_status_registers(); // вычитываем регистры
		read_control_registers();
		read_bootloader_registers();
		read_change_boot_registers();

		osDelay(1000); // ждем 1 секунду
	}
}

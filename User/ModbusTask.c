#include "cmsis_os.h"
#include "modbus.h"


extern osMessageQId ModbusQueueHandle;
extern osSemaphoreId ModbusPacketReceiveHandle;

extern uint8_t modbus_buffer[20][256];

osEvent ModbusEvent;

uint32_t crc_temp;
uint8_t modbus_packet_number = 0;
uint8_t modbus_packet_number1 = 0;


void ThreadModbusTask(void const * argument)
{
	uint16_t i=0;
	uint16_t i_max;


	for(;;)
	{
		ModbusEvent = osMessageGet(ModbusQueueHandle, osWaitForever); // ожидаем сообщение
		if(ModbusEvent.status == osEventMessage) // если сообщение пришло
		{

			switch(ModbusEvent.value.v) // проверяем полученное значение из очереди
			{
				case(0x01):
					if(i==0)
					{
						modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
						i++;
					}
					else if(i==1)
					{
						i = 0;
						i_max = 0;
					}
					else if(i>1)
					{
						if( (i==6) && (modbus_buffer[modbus_packet_number][1] == 0x10) )
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i_max = 9 + (uint8_t)ModbusEvent.value.v;
							i++;
						}
						else
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i++;
						}
					}

				break;

				case(0x03):
					if(i==0)
					{
						i = 0;
						i_max = 0;
					}
					else if(i==1)
					{
						modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
						i++;
						i_max = 8;
					}
					else if(i>1)
					{
						if( (i==6) && (modbus_buffer[modbus_packet_number][1] == 0x10) )
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i_max = 9 + (uint8_t)ModbusEvent.value.v;
							i++;
						}
						else
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i++;
						}
					}

				break;

				case(0x06):
					if(i==0)
					{
						i = 0;
						i_max = 0;
					}
					else if(i==1)
					{
						modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
						i++;
						i_max = 8;
					}
					if(i>1)
					{
						if( (i==6) && (modbus_buffer[modbus_packet_number][1] == 0x10) )
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i_max = 9 + (uint8_t)ModbusEvent.value.v;
							i++;
						}
						else
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i++;
						}
					}

				break;

				case(0x10):
					if(i==0)
					{
						i = 0;
						i_max = 0;
					}
					else if(i==1)
					{
						modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
						i++;
					}
					else if(i>1)
					{
						if( (i==6) && (modbus_buffer[modbus_packet_number][1] == 0x10) )
						{
								modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
								i_max = 9 + (uint8_t)ModbusEvent.value.v;
								i++;
						}
						else
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i++;
						}
					}

				break;

				default:
					if(i==0)
					{
						i = 0;
						i_max = 0;
					}
					else if(i==1)
					{
						i = 0;
						i_max = 0;
					}
					else if(i>1)
					{
						if( (i==6) && (modbus_buffer[modbus_packet_number][1] == 0x10) )
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i_max = 9 + (uint8_t)ModbusEvent.value.v;
							i++;
						}
						else
						{
							modbus_buffer[modbus_packet_number][i] = ModbusEvent.value.v;
							i++;
						}
					}

				break;
			}
			if( (i >= i_max) && (i != 0) && (i_max != 0) ) // если число принятых байт соответствует длине соответствующей команды
			{
				crc_temp = CRC16(&modbus_buffer[modbus_packet_number][0], i_max-2); // считаем контрольную сумму принятого пакета
				if( ( ((crc_temp>>8)&0x00FF) == modbus_buffer[modbus_packet_number][i_max-1] ) && ( (crc_temp&0x00FF) == modbus_buffer[modbus_packet_number][i_max-2]) ) // проверяем контрольную сумму принятого пакета
				{
					modbus_packet_number1 = modbus_packet_number;
					modbus_packet_number++;
					if( modbus_packet_number >= 20)
					{
						modbus_packet_number = 0;
					}

					osSemaphoreRelease(ModbusPacketReceiveHandle);
				}

				i = 0;     // обнуляем значение текущего принятого байта
				i_max = 0; // обнуляем максимальное значение принятого байта

			}
		}




		osDelay(1);
	}
}

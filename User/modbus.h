#ifndef MODBUS
#define MODBUS

#include "stm32f4xx_hal.h"

unsigned int CRC16( unsigned char * pucFrame, unsigned int usLen );


//---- Описание регистров статуса -----------------------------

#define VERSION_REG                      0x1000 // (4096) версия прошивки контроллера
#define SECURITY_STATUS_REG              0x1001 // (4097) статус режима охраны (0 - резерв ;1- включена из центра; 2 - отключена из центра; 3 - включена таблеткой 4 - отключена таблеткой; 5 - тревога открытия двери; 6 - процесс постановки на охрану; 7 - не удаётся поставить на охрану; )
#define STATUS_LOOP_REG                  0x1002 // (4098) состояние шлейфов сигнализации (номер бита соответствует номеру шлейфа (6 шлейфов) 1 - замкнут)
#define ERROR_LOOP_REG                   0x1003 // (4099) неисправные шлейфы (номер бита соответствует номеру шлейфа (6 шлейфов))
#define ALARM_LOOP_REG                   0x1004 // (4100) сработавшие шлейфы
#define TIME_CURRENT_YEAR_REG            0x1005 // (4101) текущий год
#define TIME_CURRENT_MONTH_REG           0x1006 // (4102) текущий месяц
#define TIME_CURRENT_DAY_REG             0x1007 // (4103) текущий день
#define TIME_CURRENT_HOUR_REG			 0x1008 // (4104) текущий час
#define TIME_CURRENT_MINUTE_REG          0x1009 // (4105) текущая минута
#define TIME_CURRENT_SECOND_REG          0x100A // (4106) текущая секунда
#define TIME_CURRENT_WEEKDAY_REG         0x100B // (4107) текущий день недели
#define ADDRESS_PROCESSED_EVENT_H_REG    0x100C // (4108) адрес начала записи событий
#define ADDRESS_PROCESSED_EVENT_L_REG    0x100D // (4109) адрес начала записи событий
#define ADDRESS_LAST_EVENT_H_REG         0x100E // (4110) адрес последнего записанного события
#define ADDRESS_LAST_EVENT_L_REG         0x100F // (4111) адрес последнего записанного события
#define SYSTEM_STATUS_REG                0x1010 // (4112) наименование последнего события системы (1 - включение питания; 2 - выключение питания; 3 - перезагрузка модема; 4 - перезагрузка контроллера; 5 - защитное время сеанса связи; 6 - включение сигнализации; 7 - отключение сигнализации; 8 - невозможность встать на сигнализацию; 9 - неисправность шлейфов сигнализации; 10 - тревога; 11 - пропала связь GSM;  12 - появилась связь GSM)
#define POWER_ON_REG                     0x1011 // (4113) наличие основного питания
#define ERROR_RTC_REG                    0x1012 // (4114) статус работы кварца на 32кГц
#define POWER_ON_LIGHTING_REG            0x1013 // (4115) наличие основного питания освещения

#define IBUTTON_COMPLETE_0_REG           0x1015 // (4117) нулевой байт идентифицированной таблетки
#define IBUTTON_COMPLETE_1_REG           0x1016 // (4118) первый байт идентифицированной таблетки
#define IBUTTON_COMPLETE_2_REG           0x1017 // (4119) второй байт идентифицированной таблетки
#define IBUTTON_COMPLETE_3_REG           0x1018 // (4120) третий байт идентифицированной таблетки
#define IBUTTON_COMPLETE_4_REG           0x1019 // (4121) четвертый байт идентифицированной таблетки
#define IBUTTON_COMPLETE_5_REG           0x101A // (4122) пятый байт идентифицированной таблетки
#define IBUTTON_COMPLETE_6_REG           0x101B // (4123) шестой байт идентифицированной таблетки
#define IBUTTON_COMPLETE_7_REG           0x101C // (4124) седьмой байт идентифицированной таблетки

#define CE_303_ERROR_REG                 0x101E // (4126) ошибка опроса счётчика 0 - нет ошибок 1 - таймаут 2 - ошибка данный  10 - при смене номера счётчика
#define CE_303_CURRENT_A_REG             0x101F // (4127) ток фазы А
#define CE_303_CURRENT_B_REG             0x1020 // (4128) ток фазы В
#define CE_303_CURRENT_C_REG             0x1021 // (4129) ток фазы С
#define CE_303_CURRENT_MIL_A_REG         0x1022 // (4130) милиамперы фаза А
#define CE_303_CURRENT_MIL_B_REG         0x1023 // (4131) милиамперы фаза В
#define CE_303_CURRENT_MIL_C_REG         0x1024 // (4132) милиамперы фаза С
#define CE_303_VOLT_A_REG                0x1025 // (4133) напряжение фаза А
#define CE_303_VOLT_B_REG                0x1026 // (4134) напряжение фаза В
#define CE_303_VOLT_C_REG                0x1027 // (4135) напряжение фаза С
#define CE_303_VOLT_MIL_A_REG            0x1028 // (4136) миливольты фаза А
#define CE_303_VOLT_MIL_B_REG            0x1029 // (4137) миливольты фаза В
#define CE_303_VOLT_MIL_C_REG            0x102A // (4138) миливольты фаза С
#define CE_303_POWER_A_REG               0x102B // (4139) мощность фаза А
#define CE_303_POWER_B_REG               0x102C // (4140) мощность фаза В
#define CE_303_POWER_C_REG               0x102D // (4141) мощность фаза С
#define CE_303_POWER_MIL_A_REG           0x102E // (4142) мощность миливатт фаза А
#define CE_303_POWER_MIL_B_REG           0x102F // (4143) мощность миливатт фаза В
#define CE_303_POWER_MIL_C_REG           0x1030 // (4144) мощность миливатт фаза С
#define LIGHTING_STATUS_REG              0x1031 // (4145) регистр статуса освещение
#define LIGHTING_ALARM_REG               0x1032 // (4146) регистр аварий освещение
#define MONTH_LIGHTING_OFF_REG           0x1033 // (4147) месяц выключения освещения по расписанию(последний раз)
#define DAY_LIGHTING_OFF_REG             0x1034 // (4148) день выключения освещения по расписанию(последний раз)
#define HOUR_LIGHTING_OFF_REG            0x1035 // (4149) час выключения освещения по расписанию(последний раз)
#define MINUTE_LIGHTING_OFF_REG          0x1036 // (4150) минута выключения освещения по расписанию(последний раз)
#define MONTH_LIGHTING_ON_REG            0x1037 // (4151) месяц включения освещения по расписанию(последний раз)
#define DAY_LIGHTING_ON_REG              0x1038 // (4152) день включения освещения по расписанию(последний раз)
#define HOUR_LIGHTING_ON_REG             0x1039 // (4153) час включения освещения по расписанию(последний раз)
#define MINUTE_LIGHTING_ON_REG           0x103A // (4154) минута включения освещения по расписанию(последний раз)
#define CURRENT_PHASE_A_REG              0x103B // (4155) текущий ток  фаза А *0,1А          // регистры освещения
#define CURRENT_PHASE_B_REG              0x103C // (4156) текущий ток  фаза В *0,1А          // регистры освещения
#define CURRENT_PHASE_C_REG              0x103D // (4157) текущий ток  фаза С *0,1А          // регистры освещения
#define CURRENT_VOLTAGE_A_REG            0x103E // (4158) текущее напряжение А
#define CURRENT_VOLTAGE_B_REG            0x103F // (4159) текущее напряжение В
#define CURRENT_VOLTAGE_C_REG            0x1040 // (4160) текущее напряжение С
#define CE_303_TOTAL_POWER_H_REG         0x1041 // (4161) общее показание счетчика H, старший байт   // регистры сигнализации
#define CE_303_TOTAL_POWER_L_REG         0x1042 // (4162) общее показание счетчика L, младший байт   // регистры сигнализации
#define CE_303_TOTAL_POWER_M_REG         0x1043 // (4163) общее показание счетчика M, миливатт       // регистры сигнализации
#define CE_303_TARIF1_POWER_H_REG        0x1044 // (4164) тариф 1
#define CE_303_TARIF1_POWER_L_REG        0x1045 // (4165) тариф 1
#define CE_303_TARIF1_POWER_MIL_REG      0x1046 // (4166) тариф 1
#define CE_303_TARIF2_POWER_H_REG        0x1047 // (4167) тариф 2
#define CE_303_TARIF2_POWER_L_REG        0x1048 // (4168) тариф 2
#define CE_303_TARIF2_POWER_MIL_REG      0x1049 // (4169) тариф 2
#define CE_303_TARIF3_POWER_H_REG        0x104A // (4170) тариф 3
#define CE_303_TARIF3_POWER_L_REG        0x104B // (4171) тариф 3
#define CE_303_TARIF3_POWER_MIL_REG      0x104C // (4172) тариф 3
#define CE_303_TARIF4_POWER_H_REG        0x104D // (4173) тариф 4
#define CE_303_TARIF4_POWER_L_REG        0x104E // (4174) тариф 4
#define CE_303_TARIF4_POWER_MIL_REG      0x104F // (4175) тариф 4
#define CE_303_TARIF5_POWER_H_REG        0x1050 // (4176) тариф 5
#define CE_303_TARIF5_POWER_L_REG        0x1051 // (4177) тариф 5
#define CE_303_TARIF5_POWER_MIL_REG      0x1052 // (4178) тариф 5
#define SIGNAL_LEVEL_REG                 0x1053 // (4179) уровень сигнала

#define ADVANCED_LOOP_REG                0x1055 // (4181) плата расширения, входы 1..6
#define ADDITIONAL_INPUT_REG1            0x1056 // (4182) Регистр дополнительных входов 1-8
#define ADDITIONAL_INPUT_REG2            0x1057 // (4183) Регистр дополнительных входов 9-14
#define ICCID_NUMBER_REG1                0x1058 // (4184) ICCID номер 1
#define ICCID_NUMBER_REG2                0x1059 // (4185) ICCID номер 2
#define ICCID_NUMBER_REG3                0x105A // (4186) ICCID номер 3
#define ICCID_NUMBER_REG4                0x105B // (4187) ICCID номер 4
#define ICCID_NUMBER_REG5                0x105C // (4188) ICCID номер 5
#define ICCID_NUMBER_REG6                0x105D // (4189) ICCID номер 6
#define ICCID_NUMBER_REG7                0x105E // (4190) ICCID номер 7
#define ICCID_NUMBER_REG8                0x105F // (4191) ICCID номер 8


// -------------------------------------------------------


//---- Описание регистров управления ---------------------

#define SECURITY_CONTROL_REG             0x1090 // (4240) включение охранной функции ( 0 - отключить из центра; 1- включить из центра; 2 -отключить с таблетки; 3 - включить с таблетки )
#define CONTROL_LOOP_REG                 0x1091 // (4241) перечень контролируемых шлейфов ( 1- контролировать; 0 - не контролировать (номер бита соответствует номеру шлейфа) )
#define FILTER_TIME_LOOP_REG             0x1092 // (4242) фильтр ложных срабатываний ( время срабатывания (шаг 0,01 секунды) )
#define QUANTITY_FALSE_LOOP_REG          0x1093 // (4243) количество ложных срабатываений ( количество срабатываний с временем меньше фильтра до подачи сигнала неисправность шлейфа )
#define TIME_FALSE_LOOP_REG              0x1094 // (4244) время обнуления ложных срабатываний ( время в минутах )
#define ALARM_LOOP_CLEAR_REG             0x1095 // (4245) сбросить сработавшие шлейфы ( 1 - сброс, возвращается в 0 автоматически )
#define FALSE_LOOP_CLEAR_REG             0x1096 // (4246) сброс неисправностей шлейфов
#define SECURITY_TIME_MAX_REG            0x1097 // (4247) время постановки на охрану в секундах
#define TIME_UPDATE_REG                  0x1098	// (4248) переменная установки нового времени, сбрасывается автоматически
#define TIME_YEAR_REG                    0x1099 // (4249) переменная года который необходимо установить
#define TIME_MONTH_REG                   0x109A // (4250) переменная месяца который необходимо установить
#define TIME_DAY_REG                     0x109B // (4251) переменная дня который необходимо установить
#define TIME_HOUR_REG                    0x109C // (4252) переменная часа который необходимо установить
#define TIME_MINUTE_REG                  0x109D // (4253) переменная минуты которую необходимо установить
#define TIME_SECONDS_REG                 0x109E // (4254) переменная секунд которую надо установить
#define TIME_WEEKDAY_REG                 0x109F // (4255) переменная дня недели который необходимо установить
#define MODBUS_IDLE_TIME_MAX_REG         0x10A0 // (4256) максимальное время сеанса связибез передачи данных в минутах
#define TIME_CONNECTION_TEST_REG         0x10A1 // (4257) время между тестовыми звонками на свой номер
#define EVENT_READ_REG                   0x10A2 // (4258) инициализирует чтение из памяти
#define EVENT_ADDRESS_HIGH_REG           0x10A3 // (4259) старший байт адреса чтения памяти
#define EVENT_ADDRESS_LOW_REG            0x10A4 // (4260) младший байт адреса чтения памяти
#define MODEM_RING_TRY_LOAD_REG          0x10A5 // (4261) количество попыток дозвона в центр до паузы
#define MODEM_RING_PAUSE_LOAD_REG        0x10A6 // (4262) время паузы после неудачных попыток дозвона в минутах
#define MODEM_RING_PAUSE2_LOAD_REG       0x10A7 // (4263) время между звонками в центр в секундах
#define RING_MINUTE_TIME_REG             0x10A8 // (4264) время с последнего звонка до перезагрузки модема в минутах
#define RING_HOUR_TIME_REG               0x10A9 // (4265) время с последнего звонка до перезагрузки модема в часах

#define ID_HIGH_REG                      0x10AB // (4267) ID устройства, старший байт
#define ID_LOW_REG                       0x10AC // (4268) ID устройства, младший байт
#define RESET_CONTROL_REG                0x10AD // (4269) регистр управления перезагрузкой
#define METER_POLLING_REG                0x10AE // (4270) включение опроса счётчика ( 1 - счётчик опрашивается 0 - нет )
#define IP_1_REG                         0x10AF // (4271) IP сервера
#define IP_2_REG                         0x10B0 // (4272) IP сервера
#define IP_3_REG                         0x10B1 // (4273) IP сервера
#define IP_4_REG                         0x10B2 // (4274) IP сервера
#define PORT_HIGH_REG                    0x10B3 // (4275) номер порта сервера, старший байт
#define PORT_LOW_REG                     0x10B4 // (4276) номер порта сервера, младший байт
#define METER_ID_HIGH_REG                0x10B5 // (4277) номер прибора учета, старший байт
#define METER_ID_LOW_REG                 0x10B6 // (4278) номер прибора учета, младший байт
#define GPRS_CALL_REG                    0x10B7 // (4279) флаг что устройство по GPRS звонит в центр ( 1 - звонит, сбросить в 0 )

#define MUTE_REG                         0x10B9 // (4281) регистр управления отключением звука при тревоге открытия двери (1 - звук отключен, 0 - звук включен)

#define LIGHT_CONTROL_REG                0x10C1 // (4289) регистр управления освещением (бит 0 - управление фазой А, бит 1 - управление фазой В, бит 2 - управление фазой С, бит 3- управление от каскада, бит 4 - контроль второй линии, бит 5 - работать по расписанию)
#define LIGHTING_ALARM_RESET_REG         0x10C2 // (4290) регистр сброса аварий освещения (запись 1 для сброса аварий освещения)
#define CURRENT_MEASUREMENT_ACCURACY_REG 0x10C3 // (4291) точность отслеживания тока (амперы, изменение тока приводит к сообщению в центр)
#define MAX_CURRENT_PHASE_A              0x10C4 // (4292) максимальный ток фаза А
#define MAX_CURRENT_PHASE_B              0x10C5 // (4293) максимальный ток фаза В
#define MAX_CURRENT_PHASE_C              0x10C6 // (4294) максммальный ток фаза С
#define LIGHTING_SWITCHING_REG           0x10C7 // (4295) включение функции освещения (1 - функция освещения включена, 0 - функция освещения выключена)
#define ALARM_SWITCHING_REG              0x10C8 // (4296) включение охранной функции (1 - функция охраны включена, 0 - функция охраны выключена)


//--------------------------------------------------------

//----Описание регистров бутлоадера-----------------------

#define BOOTLOADER_VERSION_REG           0x100

#define START_ADDRESS_FIRMWARE_HIGH_REG  0x102
#define START_ADDRESS_FIRMWARE_2_REG     0x103
#define START_ADDRESS_FIRMWARE_3_REG     0x104
#define START_ADDRESS_FIRMWARE_LOW_REG   0x105
#define END_ADDRESS_FIRMWARE_HIGH_REG    0x106
#define END_ADDRESS_FIRMWARE_LOW_REG     0x107
#define CRC_FIRMWARE_HIGH_REG            0x108
#define CRC_FIRMWARE_LOW_REG             0x109
#define JUMP_VECTOR_HIGH_REG             0x10A
#define JUMP_VECTOR_2_REG                0x10B
#define JUMP_VECTOR_3_REG                0x10C
#define JUMP_VECTOR_LOW_REG              0x10D
#define FIRMWARE_CORRECTNESS_REG         0x10E
#define WORKING_MODE_REG                 0x10F
#define READY_DOWNLOAD_REG               0x110
#define DOWNLOAD_TIMEOUT_REG             0x111
#define JUMP_ATTEMPT_REG                 0x112
#define MAX_JUMP_ATTEMPT_REG             0x113
#define END_ADDRESS_FIRMWARE_2_REG       0x114
#define END_ADDRESS_FIRMWARE_3_REG       0x115
#define ADDRESS_TO_WRITE_2_REG           0x116
#define ADDRESS_TO_WRITE_3_REG           0x117

#define CLEAR_PAGE_NUMBER_REG            0x11F
#define CLEAR_PAGE_ON_REG                0x120
#define WRITE_ARRAY_REG                  0x121
#define READ_ARRAY_REG                   0x122
#define ADDRESS_TO_WRITE_HIGH_REG        0x123
#define ADDRESS_TO_WRITE_LOW_REG         0x124
#define BYTE_QUANTITY_REG                0x125
#define PACKET_CRC_HIGH_REG              0x126
#define PACKET_CRC_LOW_REG               0x127
#define PACKET_DATA_0_REG                0x128
#define PACKET_DATA_1_REG                0x129
#define PACKET_DATA_2_REG                0x12A
#define PACKET_DATA_3_REG                0x12B
#define PACKET_DATA_4_REG                0x12C


//--------------------------------------------------------

//----описание регистров для смены бутлоадера-------------

#define CHANGE_BOOT_START_ADDRESS_3_REG      0x0000
#define CHANGE_BOOT_START_ADDRESS_2_REG      0x0001
#define CHANGE_BOOT_START_ADDRESS_1_REG      0x0002
#define CHANGE_BOOT_START_ADDRESS_0_REG      0x0003
#define CHANGE_BOOT_END_ADDRESS_3_REG        0x0004
#define CHANGE_BOOT_END_ADDRESS_2_REG        0x0005
#define CHANGE_BOOT_END_ADDRESS_1_REG        0x0006
#define CHANGE_BOOT_END_ADDRESS_0_REG        0x0007
#define CHANGE_BOOT_CRC_HIGH_REG             0x0008
#define CHANGE_BOOT_CRC_LOW_REG              0x0009
#define CHANGE_BOOT_ADDRESS_TO_WRITE_3_REG   0x000A
#define CHANGE_BOOT_ADDRESS_TO_WRITE_2_REG   0x000B
#define CHANGE_BOOT_ADDRESS_TO_WRITE_1_REG   0x000C
#define CHANGE_BOOT_ADDRESS_TO_WRITE_0_REG   0x000D
#define CHANGE_BOOT_CRC_CORRECTNESS_REG      0x000E
#define CHANGE_BOOT_WRITE_REG                0x000F

//--------------------------------------------------------


//----биты состояния байта SECURITY_STATUS_REG статус режима охраны------

#define RESERVED_0          0x00 // зарезервированно
#define ENABLED_BY_SERVER   0x01 // включена из центра
#define DISABLED_BY_SERVER  0x02 // отключена из центра
#define ENABLED_BY_IBUTTON  0x03 // включена таблеткой
#define DISABLED_BY_IBUTTON 0x04 // отключена таблеткой
#define DOOR_OPEN_ALARM     0x05 // тревога открытия двери
#define ARMING_PROCESS      0x06 // процесс постановки на охрану
#define ARMING_ERROR        0x07 // не удается поставить на охрану

//-----------------------------------------------------------------------

//----биты security_control регистра-------------------------------------
#define DISABLE_FROM_SERVER      0x02 // отключить из центра
#define ENABLE_FROM_SERVER       0x01 // включить из центра
#define DISABLE_FROM_IBUTTON     0x00 // отключить с таблетки
#define ENABLE_FROM_IBUTTON      0x03 // ввключить с таблетки
#define SECURITY_CONTROL_DEFAULT 0x04 // состояние покоя
//-----------------------------------------------------------------------

//----биты time_update регистра------------------------------------------
#define SET_TIME_DEFAULT 0x00
#define SET_TIME         0x01
//-----------------------------------------------------------------------

//----биты gprs_call_reg регистра------------------------------------------
#define CALL_OFF  0x00
#define CALL_ON   0x01
//-----------------------------------------------------------------------

//----биты system_status регистра------------------------------------------
#define POWER_ON     0x01
#define POWER_OFF    0x02
#define MODEM_RESET  0x03
#define MCU_RESET    0x04
#define SESSION_QUARD_TIME 0x05
#define TURN_ON_STATE_ALARM 0x06
#define TURN_OFF_STATE_ALARM 0x07
#define TURN_ON_ALARM_ERROR 0x08
#define ERROR_LOOP 0x09
#define ALARM_STATE 0x0A
#define LOST_GSM_CONNECTION 0x0B
#define RESTORED_GSM_CONNECTION 0x0C
//-----------------------------------------------------------------------

//----биты mute_reg регистра------------------------------------------
#define MUTE_OFF  0x00
#define MUTE_ON   0x01
//-----------------------------------------------------------------------

//----биты LIGHTING_SWITCHING_REG регистра, включение функции освещения----
#define LIGHTING_ON  0x01 // функция освещения включена
#define LIGHTING_OFF 0x00 // функция освещения выключена
//-------------------------------------------------------------------------

//----биты ALARM_SWITCHING_REG регистра, включение охранной функции--------
#define ALARM_ON  0x01 // охранная функция включена
#define ALARM_OFF 0x00 // охранная функция выключена
//-------------------------------------------------------------------------

//----биты LIGHT_CONTROL_REG регистра-------------------------------
#define PHASE_A_SWITCH_OFF      0x00
#define PHASE_A_SWITCH_ON       0x01
#define PHASE_A_SWITCH_DEFAULT  0x02
#define PHASE_B_SWITCH_OFF      0x00
#define PHASE_B_SWITCH_ON       0x02
#define PHASE_C_SWITCH_OFF      0x00
#define PHASE_C_SWITCH_ON       0x04
//------------------------------------------------------------------

//----структура переменной статусных регистров---------------------
typedef struct
{
	uint16_t version_reg;
	uint16_t security_status_reg;
	uint16_t status_loop_reg;
	uint16_t error_loop_reg;
	uint16_t alarm_loop_reg;
	uint16_t time_current_year_reg;
	uint16_t time_current_month_reg;
	uint16_t time_current_day_reg;
	uint16_t time_current_hour_reg;
	uint16_t time_current_minute_reg;
	uint16_t time_current_second_reg;
	uint16_t time_current_weekday_reg;
	uint16_t address_processed_event_h_reg;
	uint16_t address_processed_event_l_reg;
	uint16_t address_last_event_h_reg;
	uint16_t address_last_event_l_reg;
	uint16_t system_status_reg;
	uint16_t power_on_reg;
	uint16_t error_rtc_reg;
	uint16_t power_on_lighting_reg;
	uint16_t reserved_2;
	uint16_t ibutton_complite_0_reg;
	uint16_t ibutton_complite_1_reg;
	uint16_t ibutton_complite_2_reg;
	uint16_t ibutton_complite_3_reg;
	uint16_t ibutton_complite_4_reg;
	uint16_t ibutton_complite_5_reg;
	uint16_t ibutton_complite_6_reg;
	uint16_t ibutton_complite_7_reg;
	uint16_t reserved_3;
	uint16_t ce303_error_reg;
	uint16_t ce303_current_a_reg;
	uint16_t ce303_current_b_reg;
	uint16_t ce303_current_c_reg;
	uint16_t ce303_current_mil_a_reg;
	uint16_t ce303_current_mil_b_reg;
	uint16_t ce303_current_mil_c_reg;
	uint16_t ce303_volt_a_reg;
	uint16_t ce303_volt_b_reg;
	uint16_t ce303_volt_c_reg;
	uint16_t ce303_volt_mil_a_reg;
	uint16_t ce303_volt_mil_b_reg;
	uint16_t ce303_volt_mil_c_reg;

	uint16_t ce303_power_a_reg;
	uint16_t ce303_power_b_reg;
	uint16_t ce303_power_c_reg;
	uint16_t ce303_power_mil_a_reg;
	uint16_t ce303_power_mil_b_reg;
	uint16_t ce303_power_mil_c_reg;

	uint16_t lighting_status_reg;
	uint16_t lighting_alarm_reg;
	uint16_t month_lighting_off_reg;
	uint16_t day_lighting_off_reg;
	uint16_t hour_lighting_off_reg;
	uint16_t minute_lighting_off_reg;
	uint16_t month_lighting_on_reg;
	uint16_t day_lighting_on_reg;
	uint16_t hour_lighting_on_reg;
	uint16_t minute_lighting_on_reg;
	uint16_t current_phase_a_reg;
	uint16_t current_phase_b_reg;
	uint16_t current_phase_c_reg;
	uint16_t current_voltage_a_reg;
	uint16_t current_voltage_b_reg;
	uint16_t current_voltage_c_reg;
	uint16_t ce303_total_power_h_reg;
	uint16_t ce303_total_power_l_reg;
	uint16_t ce303_total_power_m_reg;
	uint16_t ce303_tarif1_power_h_reg;
	uint16_t ce303_tarif1_power_l_reg;
	uint16_t ce303_tarif1_power_mil_reg;
	uint16_t ce303_tarif2_power_h_reg;
	uint16_t ce303_tarif2_power_l_reg;
	uint16_t ce303_tarif2_power_mil_reg;
	uint16_t ce303_tarif3_power_h_reg;
	uint16_t ce303_tarif3_power_l_reg;
	uint16_t ce303_tarif3_power_mil_reg;
	uint16_t ce303_tarif4_power_h_reg;
	uint16_t ce303_tarif4_power_l_reg;
	uint16_t ce303_tarif4_power_mil_reg;
	uint16_t ce303_tarif5_power_h_reg;
	uint16_t ce303_tarif5_power_l_reg;
	uint16_t ce303_tarif5_power_mil_reg;
	uint16_t signal_level_reg;

	uint16_t advanced_loop_reg;
	uint16_t additional_input_reg1;
	uint16_t additional_input_reg2;
	uint16_t iccid_number_reg1;
	uint16_t iccid_number_reg2;
	uint16_t iccid_number_reg3;
	uint16_t iccid_number_reg4;
	uint16_t iccid_number_reg5;
	uint16_t iccid_number_reg6;
	uint16_t iccid_number_reg7;
	uint16_t iccid_number_reg8;

} status_register_struct;
//------------------------------------------------------------------

//----структура переменной управляющих регистров--------------------
typedef struct
{
	uint16_t security_control_reg;
	uint16_t control_loop_reg;
	uint16_t filter_time_loop_reg;
	uint16_t quantity_false_loop_reg;
	uint16_t time_false_loop_reg;
	uint16_t alarm_loop_clear_reg;
	uint16_t false_loop_clear_reg;
	uint16_t security_time_max_reg;
	uint16_t time_update_reg;
	uint16_t time_year_reg;
	uint16_t time_month_reg;
	uint16_t time_day_reg;
	uint16_t time_hour_reg;
	uint16_t time_minute_reg;
	uint16_t time_seconds_reg;
	uint16_t time_weekday_reg;
	uint16_t modbus_idle_time_max_reg;
	uint16_t time_connection_test_reg;
	uint16_t event_read_reg;
	uint16_t event_address_high_reg;
	uint16_t event_address_low_reg;
	uint16_t modem_ring_try_load_reg;
	uint16_t modem_ring_pause_load_reg;
	uint16_t modem_ring_pause2_load_reg;
	uint16_t ring_minute_time_reg;
	uint16_t ring_hour_time_reg;

	uint16_t id_high_reg;
	uint16_t id_low_reg;
	uint16_t reset_control_reg;
	uint16_t meter_polling_reg;
	uint16_t ip1_reg;
	uint16_t ip2_reg;
	uint16_t ip3_reg;
	uint16_t ip4_reg;
	uint16_t port_high_reg;
	uint16_t port_low_reg;
	uint16_t meter_id_high_reg;
	uint16_t meter_id_low_reg;
	uint16_t gprs_call_reg;

	uint16_t mute_reg; // переменная управления отключением звука при тревоге открытия двери

	uint16_t light_control_reg; // переменная управления освещением
	uint16_t lighting_alarm_reset_reg;
	uint16_t current_measurement_accuracy_reg;
	uint16_t max_current_phase_a;
	uint16_t max_current_phase_b;
	uint16_t max_current_phase_c;
	uint16_t lighting_switching_reg;
	uint16_t alarm_switching_reg;


} control_register_struct;
//------------------------------------------------------------------

//----структура переменной регистров загрузчика---------------------

typedef struct
{
	uint16_t bootloader_version_reg;
	uint16_t start_address_firmware_high_reg;
	uint16_t start_address_firmware_2_reg;
	uint16_t start_address_firmware_3_reg;
	uint16_t start_address_firmware_low_reg;
	uint16_t end_address_firmware_high_reg;
	uint16_t end_address_firmware_low_reg;
	uint16_t crc_firmware_high_reg;
	uint16_t crc_firmware_low_reg;
	uint16_t jump_vector_high_reg;
	uint16_t jump_vector_2_reg;
	uint16_t jump_vector_3_reg;
	uint16_t jump_vector_low_reg;
	uint16_t firmware_correctness_reg;
	uint16_t working_mode_reg;
	uint16_t ready_download_reg;
	uint16_t download_timeout_reg;
	uint16_t jump_attempt_reg;
	uint16_t max_jump_attempt_reg;
	uint16_t end_address_firmware_2_reg;
	uint16_t end_address_firmware_3_reg;
	uint16_t address_to_write_2_reg;
	uint16_t address_to_write_3_reg;

	uint16_t clear_page_number_reg;
	uint16_t clear_page_on_reg;
	uint16_t write_array_reg;
	uint16_t read_array_reg;
	uint16_t address_to_write_high_reg;
	uint16_t address_to_write_low_reg;
	uint16_t byte_quantity_reg;
	uint16_t packet_crc_high_reg;
	uint16_t packet_crc_low_reg;
	uint16_t packet_data_0_reg;
	uint16_t packet_data_1_reg;
	uint16_t packet_data_2_reg;
	uint16_t packet_data_3_reg;
	uint16_t packet_data_4_reg;


} bootloader_register_struct;
//------------------------------------------------------------------

//----структура переменной регистров смены загрузчика---------------

typedef struct
{

	uint16_t change_boot_start_address_3_reg;
	uint16_t change_boot_start_address_2_reg;
	uint16_t change_boot_start_address_1_reg;
	uint16_t change_boot_start_address_0_reg;
	uint16_t change_boot_end_address_3_reg;
	uint16_t change_boot_end_address_2_reg;
	uint16_t change_boot_end_address_1_reg;
	uint16_t change_boot_end_address_0_reg;
	uint16_t change_boot_crc_high_reg;
	uint16_t change_boot_crc_low_reg;
	uint16_t change_boot_address_to_write_3_reg;
	uint16_t change_boot_address_to_write_2_reg;
	uint16_t change_boot_address_to_write_1_reg;
	uint16_t change_boot_address_to_write_0_reg;
	uint16_t change_boot_crc_correctness_reg;
	uint16_t change_boot_write_reg;

} change_boot_register_struct;

//------------------------------------------------------------------


void read_status_registers(void);
void read_control_registers(void);
void read_bootloader_registers(void);
void read_bootloader_registers_no_rtos(void);
void read_change_boot_registers(void);



#endif

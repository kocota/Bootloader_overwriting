#include "cmsis_os.h"
#include "modbus.h"
#include "fm25v02.h"


extern osMutexId Fm25v02MutexHandle;

status_register_struct status_registers;
control_register_struct control_registers;
bootloader_register_struct bootloader_registers;
change_boot_register_struct change_boot_registers;

//----функция чтения из памяти регистров статуса--------------------------
void read_status_registers(void)
{
	uint8_t status_reg_temp_h;
	uint8_t status_reg_temp_l;

	osMutexWait(Fm25v02MutexHandle, osWaitForever);

	fm25v02_read(2*VERSION_REG, &status_reg_temp_h);
	fm25v02_read(2*VERSION_REG+1, &status_reg_temp_l);
	status_registers.version_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*SECURITY_STATUS_REG, &status_reg_temp_h);
	fm25v02_read(2*SECURITY_STATUS_REG+1, &status_reg_temp_l);
	status_registers.security_status_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*STATUS_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*STATUS_LOOP_REG+1, &status_reg_temp_l);
	status_registers.status_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ERROR_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*ERROR_LOOP_REG+1, &status_reg_temp_l);
	status_registers.error_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ALARM_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*ALARM_LOOP_REG+1, &status_reg_temp_l);
	status_registers.alarm_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_YEAR_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_YEAR_REG+1, &status_reg_temp_l);
	status_registers.time_current_year_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_MONTH_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_MONTH_REG+1, &status_reg_temp_l);
	status_registers.time_current_month_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_DAY_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_DAY_REG+1, &status_reg_temp_l);
	status_registers.time_current_day_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_HOUR_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_HOUR_REG+1, &status_reg_temp_l);
	status_registers.time_current_hour_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_MINUTE_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_MINUTE_REG+1, &status_reg_temp_l);
	status_registers.time_current_minute_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_SECOND_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_SECOND_REG+1, &status_reg_temp_l);
	status_registers.time_current_second_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CURRENT_WEEKDAY_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CURRENT_WEEKDAY_REG+1, &status_reg_temp_l);
	status_registers.time_current_weekday_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDRESS_PROCESSED_EVENT_H_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_PROCESSED_EVENT_H_REG+1, &status_reg_temp_l);
	status_registers.address_processed_event_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDRESS_PROCESSED_EVENT_L_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_PROCESSED_EVENT_L_REG+1, &status_reg_temp_l);
	status_registers.address_processed_event_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDRESS_LAST_EVENT_H_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_LAST_EVENT_H_REG+1, &status_reg_temp_l);
	status_registers.address_last_event_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDRESS_LAST_EVENT_L_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_LAST_EVENT_L_REG+1, &status_reg_temp_l);
	status_registers.address_last_event_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*SYSTEM_STATUS_REG, &status_reg_temp_h);
	fm25v02_read(2*SYSTEM_STATUS_REG+1, &status_reg_temp_l);
	status_registers.system_status_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*POWER_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*POWER_ON_REG+1, &status_reg_temp_l);
	status_registers.power_on_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ERROR_RTC_REG, &status_reg_temp_h);
	fm25v02_read(2*ERROR_RTC_REG+1, &status_reg_temp_l);
	status_registers.error_rtc_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*POWER_ON_LIGHTING_REG, &status_reg_temp_h);
	fm25v02_read(2*POWER_ON_LIGHTING_REG+1, &status_reg_temp_l);
	status_registers.power_on_lighting_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_0_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_0_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_0_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_1_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_1_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_1_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_2_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_2_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_3_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_3_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_4_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_4_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_4_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_5_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_5_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_5_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_6_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_6_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_6_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IBUTTON_COMPLETE_7_REG, &status_reg_temp_h);
	fm25v02_read(2*IBUTTON_COMPLETE_7_REG+1, &status_reg_temp_l);
	status_registers.ibutton_complite_7_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_ERROR_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_ERROR_REG+1, &status_reg_temp_l);
	status_registers.ce303_error_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_MIL_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_MIL_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_mil_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_MIL_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_MIL_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_mil_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_CURRENT_MIL_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_CURRENT_MIL_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_current_mil_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_MIL_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_MIL_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_mil_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_MIL_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_MIL_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_mil_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_VOLT_MIL_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_VOLT_MIL_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_volt_mil_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_MIL_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_MIL_A_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_mil_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_MIL_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_MIL_B_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_mil_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_POWER_MIL_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_POWER_MIL_C_REG+1, &status_reg_temp_l);
	status_registers.ce303_power_mil_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*LIGHTING_STATUS_REG, &status_reg_temp_h);
	fm25v02_read(2*LIGHTING_STATUS_REG+1, &status_reg_temp_l);
	status_registers.lighting_status_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*LIGHTING_ALARM_REG, &status_reg_temp_h);
	fm25v02_read(2*LIGHTING_ALARM_REG+1, &status_reg_temp_l);
	status_registers.lighting_alarm_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MONTH_LIGHTING_OFF_REG, &status_reg_temp_h);
	fm25v02_read(2*MONTH_LIGHTING_OFF_REG+1, &status_reg_temp_l);
	status_registers.month_lighting_off_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*DAY_LIGHTING_OFF_REG, &status_reg_temp_h);
	fm25v02_read(2*DAY_LIGHTING_OFF_REG+1, &status_reg_temp_l);
	status_registers.day_lighting_off_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*HOUR_LIGHTING_OFF_REG, &status_reg_temp_h);
	fm25v02_read(2*HOUR_LIGHTING_OFF_REG+1, &status_reg_temp_l);
	status_registers.hour_lighting_off_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MINUTE_LIGHTING_OFF_REG, &status_reg_temp_h);
	fm25v02_read(2*MINUTE_LIGHTING_OFF_REG+1, &status_reg_temp_l);
	status_registers.minute_lighting_off_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MONTH_LIGHTING_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*MONTH_LIGHTING_ON_REG+1, &status_reg_temp_l);
	status_registers.month_lighting_on_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*DAY_LIGHTING_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*DAY_LIGHTING_ON_REG+1, &status_reg_temp_l);
	status_registers.day_lighting_on_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*HOUR_LIGHTING_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*HOUR_LIGHTING_ON_REG+1, &status_reg_temp_l);
	status_registers.hour_lighting_on_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MINUTE_LIGHTING_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*MINUTE_LIGHTING_ON_REG+1, &status_reg_temp_l);
	status_registers.minute_lighting_on_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_PHASE_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_PHASE_A_REG+1, &status_reg_temp_l);
	status_registers.current_phase_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_PHASE_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_PHASE_B_REG+1, &status_reg_temp_l);
	status_registers.current_phase_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_PHASE_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_PHASE_C_REG+1, &status_reg_temp_l);
	status_registers.current_phase_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_VOLTAGE_A_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_VOLTAGE_A_REG+1, &status_reg_temp_l);
	status_registers.current_voltage_a_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_VOLTAGE_B_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_VOLTAGE_B_REG+1, &status_reg_temp_l);
	status_registers.current_voltage_b_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_VOLTAGE_C_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_VOLTAGE_C_REG+1, &status_reg_temp_l);
	status_registers.current_voltage_c_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TOTAL_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TOTAL_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_total_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TOTAL_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TOTAL_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_total_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TOTAL_POWER_M_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TOTAL_POWER_M_REG+1, &status_reg_temp_l);
	status_registers.ce303_total_power_m_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF1_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF1_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif1_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF1_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF1_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif1_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF1_POWER_MIL_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF1_POWER_MIL_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif1_power_mil_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF2_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF2_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif2_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF2_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF2_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif2_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF2_POWER_MIL_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF2_POWER_MIL_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif2_power_mil_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF3_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF3_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif3_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF3_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF3_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif3_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF3_POWER_MIL_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF3_POWER_MIL_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif3_power_mil_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF4_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF4_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif4_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF4_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF4_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif4_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF4_POWER_MIL_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF4_POWER_MIL_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif4_power_mil_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF5_POWER_H_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF5_POWER_H_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif5_power_h_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF5_POWER_L_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF5_POWER_L_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif5_power_l_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CE_303_TARIF5_POWER_MIL_REG, &status_reg_temp_h);
	fm25v02_read(2*CE_303_TARIF5_POWER_MIL_REG+1, &status_reg_temp_l);
	status_registers.ce303_tarif5_power_mil_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*SIGNAL_LEVEL_REG, &status_reg_temp_h);
	fm25v02_read(2*SIGNAL_LEVEL_REG+1, &status_reg_temp_l);
	status_registers.signal_level_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADVANCED_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*ADVANCED_LOOP_REG+1, &status_reg_temp_l);
	status_registers.advanced_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDITIONAL_INPUT_REG1, &status_reg_temp_h);
	fm25v02_read(2*ADDITIONAL_INPUT_REG1+1, &status_reg_temp_l);
	status_registers.additional_input_reg1 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ADDITIONAL_INPUT_REG2, &status_reg_temp_h);
	fm25v02_read(2*ADDITIONAL_INPUT_REG2+1, &status_reg_temp_l);
	status_registers.additional_input_reg2 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG1, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG1+1, &status_reg_temp_l);
	status_registers.iccid_number_reg1 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG2, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG2+1, &status_reg_temp_l);
	status_registers.iccid_number_reg2 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG3, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG3+1, &status_reg_temp_l);
	status_registers.iccid_number_reg3 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG4, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG4+1, &status_reg_temp_l);
	status_registers.iccid_number_reg4 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG5, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG5+1, &status_reg_temp_l);
	status_registers.iccid_number_reg5 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG6, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG6+1, &status_reg_temp_l);
	status_registers.iccid_number_reg6 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG7, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG7+1, &status_reg_temp_l);
	status_registers.iccid_number_reg7 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ICCID_NUMBER_REG8, &status_reg_temp_h);
	fm25v02_read(2*ICCID_NUMBER_REG8+1, &status_reg_temp_l);
	status_registers.iccid_number_reg8 = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	osMutexRelease(Fm25v02MutexHandle);
}
//----------------------------------------------------------------

//----Функция чтения из памяти управляющих регистров--------------
void read_control_registers(void)
{
	uint8_t status_reg_temp_h;
	uint8_t status_reg_temp_l;

	osMutexWait(Fm25v02MutexHandle, osWaitForever);

	fm25v02_read(2*SECURITY_CONTROL_REG, &status_reg_temp_h);
	fm25v02_read(2*SECURITY_CONTROL_REG+1, &status_reg_temp_l);
	control_registers.security_control_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CONTROL_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*CONTROL_LOOP_REG+1, &status_reg_temp_l);
	control_registers.control_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*FILTER_TIME_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*FILTER_TIME_LOOP_REG+1, &status_reg_temp_l);
	control_registers.filter_time_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*QUANTITY_FALSE_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*QUANTITY_FALSE_LOOP_REG+1, &status_reg_temp_l);
	control_registers.quantity_false_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_FALSE_LOOP_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_FALSE_LOOP_REG+1, &status_reg_temp_l);
	control_registers.time_false_loop_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ALARM_LOOP_CLEAR_REG, &status_reg_temp_h);
	fm25v02_read(2*ALARM_LOOP_CLEAR_REG+1, &status_reg_temp_l);
	control_registers.alarm_loop_clear_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*FALSE_LOOP_CLEAR_REG, &status_reg_temp_h);
	fm25v02_read(2*FALSE_LOOP_CLEAR_REG+1, &status_reg_temp_l);
	control_registers.false_loop_clear_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*SECURITY_TIME_MAX_REG, &status_reg_temp_h);
	fm25v02_read(2*SECURITY_TIME_MAX_REG+1, &status_reg_temp_l);
	control_registers.security_time_max_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_UPDATE_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_UPDATE_REG+1, &status_reg_temp_l);
	control_registers.time_update_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_YEAR_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_YEAR_REG+1, &status_reg_temp_l);
	control_registers.time_year_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_MONTH_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_MONTH_REG+1, &status_reg_temp_l);
	control_registers.time_month_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_DAY_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_DAY_REG+1, &status_reg_temp_l);
	control_registers.time_day_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_HOUR_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_HOUR_REG+1, &status_reg_temp_l);
	control_registers.time_hour_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_MINUTE_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_MINUTE_REG+1, &status_reg_temp_l);
	control_registers.time_minute_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_SECONDS_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_SECONDS_REG+1, &status_reg_temp_l);
	control_registers.time_seconds_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_WEEKDAY_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_WEEKDAY_REG+1, &status_reg_temp_l);
	control_registers.time_weekday_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MODBUS_IDLE_TIME_MAX_REG, &status_reg_temp_h);
	fm25v02_read(2*MODBUS_IDLE_TIME_MAX_REG+1, &status_reg_temp_l);
	control_registers.modbus_idle_time_max_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*TIME_CONNECTION_TEST_REG, &status_reg_temp_h);
	fm25v02_read(2*TIME_CONNECTION_TEST_REG+1, &status_reg_temp_l);
	control_registers.time_connection_test_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*EVENT_READ_REG, &status_reg_temp_h);
	fm25v02_read(2*EVENT_READ_REG+1, &status_reg_temp_l);
	control_registers.event_read_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*EVENT_ADDRESS_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*EVENT_ADDRESS_HIGH_REG+1, &status_reg_temp_l);
	control_registers.event_address_high_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*EVENT_ADDRESS_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*EVENT_ADDRESS_LOW_REG+1, &status_reg_temp_l);
	control_registers.event_address_low_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MODEM_RING_TRY_LOAD_REG, &status_reg_temp_h);
	fm25v02_read(2*MODEM_RING_TRY_LOAD_REG+1, &status_reg_temp_l);
	control_registers.modem_ring_try_load_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MODEM_RING_PAUSE_LOAD_REG, &status_reg_temp_h);
	fm25v02_read(2*MODEM_RING_PAUSE_LOAD_REG+1, &status_reg_temp_l);
	control_registers.modem_ring_pause_load_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MODEM_RING_PAUSE2_LOAD_REG, &status_reg_temp_h);
	fm25v02_read(2*MODEM_RING_PAUSE2_LOAD_REG+1, &status_reg_temp_l);
	control_registers.modem_ring_pause2_load_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*RING_MINUTE_TIME_REG, &status_reg_temp_h);
	fm25v02_read(2*RING_MINUTE_TIME_REG+1, &status_reg_temp_l);
	control_registers.ring_minute_time_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*RING_HOUR_TIME_REG, &status_reg_temp_h);
	fm25v02_read(2*RING_HOUR_TIME_REG+1, &status_reg_temp_l);
	control_registers.ring_hour_time_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ID_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*ID_HIGH_REG+1, &status_reg_temp_l);
	control_registers.id_high_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ID_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*ID_LOW_REG+1, &status_reg_temp_l);
	control_registers.id_low_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*RESET_CONTROL_REG, &status_reg_temp_h);
	fm25v02_read(2*RESET_CONTROL_REG+1, &status_reg_temp_l);
	control_registers.reset_control_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*METER_POLLING_REG, &status_reg_temp_h);
	fm25v02_read(2*METER_POLLING_REG+1, &status_reg_temp_l);
	control_registers.meter_polling_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IP_1_REG, &status_reg_temp_h);
	fm25v02_read(2*IP_1_REG+1, &status_reg_temp_l);
	control_registers.ip1_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IP_2_REG, &status_reg_temp_h);
	fm25v02_read(2*IP_2_REG+1, &status_reg_temp_l);
	control_registers.ip2_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IP_3_REG, &status_reg_temp_h);
	fm25v02_read(2*IP_3_REG+1, &status_reg_temp_l);
	control_registers.ip3_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*IP_4_REG, &status_reg_temp_h);
	fm25v02_read(2*IP_4_REG+1, &status_reg_temp_l);
	control_registers.ip4_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*PORT_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*PORT_HIGH_REG+1, &status_reg_temp_l);
	control_registers.port_high_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*PORT_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*PORT_LOW_REG+1, &status_reg_temp_l);
	control_registers.port_low_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*METER_ID_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*METER_ID_HIGH_REG+1, &status_reg_temp_l);
	control_registers.meter_id_high_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*METER_ID_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*METER_ID_LOW_REG+1, &status_reg_temp_l);
	control_registers.meter_id_low_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*GPRS_CALL_REG, &status_reg_temp_h);
	fm25v02_read(2*GPRS_CALL_REG+1, &status_reg_temp_l);
	control_registers.gprs_call_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MUTE_REG, &status_reg_temp_h);
	fm25v02_read(2*MUTE_REG+1, &status_reg_temp_l);
	control_registers.mute_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*LIGHT_CONTROL_REG, &status_reg_temp_h);
	fm25v02_read(2*LIGHT_CONTROL_REG+1, &status_reg_temp_l);
	control_registers.light_control_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*LIGHTING_ALARM_RESET_REG, &status_reg_temp_h);
	fm25v02_read(2*LIGHTING_ALARM_RESET_REG+1, &status_reg_temp_l);
	control_registers.lighting_alarm_reset_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*CURRENT_MEASUREMENT_ACCURACY_REG, &status_reg_temp_h);
	fm25v02_read(2*CURRENT_MEASUREMENT_ACCURACY_REG+1, &status_reg_temp_l);
	control_registers.current_measurement_accuracy_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MAX_CURRENT_PHASE_A, &status_reg_temp_h);
	fm25v02_read(2*MAX_CURRENT_PHASE_A+1, &status_reg_temp_l);
	control_registers.max_current_phase_a = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MAX_CURRENT_PHASE_B, &status_reg_temp_h);
	fm25v02_read(2*MAX_CURRENT_PHASE_B+1, &status_reg_temp_l);
	control_registers.max_current_phase_b = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*MAX_CURRENT_PHASE_C, &status_reg_temp_h);
	fm25v02_read(2*MAX_CURRENT_PHASE_C+1, &status_reg_temp_l);
	control_registers.max_current_phase_c = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*LIGHTING_SWITCHING_REG, &status_reg_temp_h);
	fm25v02_read(2*LIGHTING_SWITCHING_REG+1, &status_reg_temp_l);
	control_registers.lighting_switching_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	fm25v02_read(2*ALARM_SWITCHING_REG, &status_reg_temp_h);
	fm25v02_read(2*ALARM_SWITCHING_REG+1, &status_reg_temp_l);
	control_registers.alarm_switching_reg = (((uint16_t)status_reg_temp_h)<<8)|status_reg_temp_l;

	osMutexRelease(Fm25v02MutexHandle);

}
//----------------------------------------------------------------



//----Функция чтения из памяти регистров бутлоадера---------------

void read_bootloader_registers(void)
{
	uint8_t status_reg_temp_h;
	uint8_t status_reg_temp_l;

	osMutexWait(Fm25v02MutexHandle, osWaitForever);

	fm25v02_read(2*BOOTLOADER_VERSION_REG, &status_reg_temp_h);
	fm25v02_read(2*BOOTLOADER_VERSION_REG+1, &status_reg_temp_l);
	bootloader_registers.bootloader_version_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CRC_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*CRC_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.crc_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CRC_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*CRC_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.crc_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_2_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_2_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_3_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_3_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*FIRMWARE_CORRECTNESS_REG, &status_reg_temp_h);
	fm25v02_read(2*FIRMWARE_CORRECTNESS_REG+1, &status_reg_temp_l);
	bootloader_registers.firmware_correctness_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*WORKING_MODE_REG, &status_reg_temp_h);
	fm25v02_read(2*WORKING_MODE_REG+1, &status_reg_temp_l);
	bootloader_registers.working_mode_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*READY_DOWNLOAD_REG, &status_reg_temp_h);
	fm25v02_read(2*READY_DOWNLOAD_REG+1, &status_reg_temp_l);
	bootloader_registers.ready_download_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*DOWNLOAD_TIMEOUT_REG, &status_reg_temp_h);
	fm25v02_read(2*DOWNLOAD_TIMEOUT_REG+1, &status_reg_temp_l);
	bootloader_registers.download_timeout_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_ATTEMPT_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_ATTEMPT_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_attempt_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*MAX_JUMP_ATTEMPT_REG, &status_reg_temp_h);
	fm25v02_read(2*MAX_JUMP_ATTEMPT_REG+1, &status_reg_temp_l);
	bootloader_registers.max_jump_attempt_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CLEAR_PAGE_NUMBER_REG, &status_reg_temp_h);
	fm25v02_read(2*CLEAR_PAGE_NUMBER_REG+1, &status_reg_temp_l);
	bootloader_registers.clear_page_number_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CLEAR_PAGE_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*CLEAR_PAGE_ON_REG+1, &status_reg_temp_l);
	bootloader_registers.clear_page_on_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*WRITE_ARRAY_REG, &status_reg_temp_h);
	fm25v02_read(2*WRITE_ARRAY_REG+1, &status_reg_temp_l);
	bootloader_registers.write_array_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*READ_ARRAY_REG, &status_reg_temp_h);
	fm25v02_read(2*READ_ARRAY_REG+1, &status_reg_temp_l);
	bootloader_registers.read_array_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*BYTE_QUANTITY_REG, &status_reg_temp_h);
	fm25v02_read(2*BYTE_QUANTITY_REG+1, &status_reg_temp_l);
	bootloader_registers.byte_quantity_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_CRC_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_CRC_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_crc_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_CRC_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_CRC_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_crc_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_0_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_0_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_0_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_1_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_1_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_1_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_2_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_2_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_3_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_3_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_4_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_4_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_4_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	osMutexRelease(Fm25v02MutexHandle);

}
//----------------------------------------------------------------

//----Функция чтения из памяти регистров бутлоадера без запуска операционной системы---------------

void read_bootloader_registers_no_rtos(void)
{
	uint8_t status_reg_temp_h;
	uint8_t status_reg_temp_l;

	fm25v02_read(2*BOOTLOADER_VERSION_REG, &status_reg_temp_h);
	fm25v02_read(2*BOOTLOADER_VERSION_REG+1, &status_reg_temp_l);
	bootloader_registers.bootloader_version_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*START_ADDRESS_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*START_ADDRESS_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.start_address_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CRC_FIRMWARE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*CRC_FIRMWARE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.crc_firmware_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CRC_FIRMWARE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*CRC_FIRMWARE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.crc_firmware_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_2_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_2_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_3_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_3_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_VECTOR_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_VECTOR_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_vector_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*FIRMWARE_CORRECTNESS_REG, &status_reg_temp_h);
	fm25v02_read(2*FIRMWARE_CORRECTNESS_REG+1, &status_reg_temp_l);
	bootloader_registers.firmware_correctness_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*WORKING_MODE_REG, &status_reg_temp_h);
	fm25v02_read(2*WORKING_MODE_REG+1, &status_reg_temp_l);
	bootloader_registers.working_mode_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*READY_DOWNLOAD_REG, &status_reg_temp_h);
	fm25v02_read(2*READY_DOWNLOAD_REG+1, &status_reg_temp_l);
	bootloader_registers.ready_download_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*DOWNLOAD_TIMEOUT_REG, &status_reg_temp_h);
	fm25v02_read(2*DOWNLOAD_TIMEOUT_REG+1, &status_reg_temp_l);
	bootloader_registers.download_timeout_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*JUMP_ATTEMPT_REG, &status_reg_temp_h);
	fm25v02_read(2*JUMP_ATTEMPT_REG+1, &status_reg_temp_l);
	bootloader_registers.jump_attempt_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*MAX_JUMP_ATTEMPT_REG, &status_reg_temp_h);
	fm25v02_read(2*MAX_JUMP_ATTEMPT_REG+1, &status_reg_temp_l);
	bootloader_registers.max_jump_attempt_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*END_ADDRESS_FIRMWARE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*END_ADDRESS_FIRMWARE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.end_address_firmware_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_2_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_3_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CLEAR_PAGE_NUMBER_REG, &status_reg_temp_h);
	fm25v02_read(2*CLEAR_PAGE_NUMBER_REG+1, &status_reg_temp_l);
	bootloader_registers.clear_page_number_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CLEAR_PAGE_ON_REG, &status_reg_temp_h);
	fm25v02_read(2*CLEAR_PAGE_ON_REG+1, &status_reg_temp_l);
	bootloader_registers.clear_page_on_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*WRITE_ARRAY_REG, &status_reg_temp_h);
	fm25v02_read(2*WRITE_ARRAY_REG+1, &status_reg_temp_l);
	bootloader_registers.write_array_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*READ_ARRAY_REG, &status_reg_temp_h);
	fm25v02_read(2*READ_ARRAY_REG+1, &status_reg_temp_l);
	bootloader_registers.read_array_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*ADDRESS_TO_WRITE_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*ADDRESS_TO_WRITE_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.address_to_write_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*BYTE_QUANTITY_REG, &status_reg_temp_h);
	fm25v02_read(2*BYTE_QUANTITY_REG+1, &status_reg_temp_l);
	bootloader_registers.byte_quantity_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_CRC_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_CRC_HIGH_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_crc_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_CRC_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_CRC_LOW_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_crc_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_0_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_0_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_0_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_1_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_1_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_1_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_2_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_2_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_3_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_3_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*PACKET_DATA_4_REG, &status_reg_temp_h);
	fm25v02_read(2*PACKET_DATA_4_REG+1, &status_reg_temp_l);
	bootloader_registers.packet_data_4_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

}
//----------------------------------------------------------------

//----Функция чтения из памяти регистров бутлоадера без запуска операционной системы---------------

void read_change_boot_registers(void)
{
	uint8_t status_reg_temp_h;
	uint8_t status_reg_temp_l;

	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_3_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_3_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_start_address_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_2_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_2_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_start_address_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_1_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_1_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_start_address_1_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_0_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_START_ADDRESS_0_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_start_address_0_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_3_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_3_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_end_address_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_2_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_2_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_end_address_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_1_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_1_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_end_address_1_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_0_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_END_ADDRESS_0_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_end_address_0_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_CRC_HIGH_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_CRC_HIGH_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_crc_high_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_CRC_LOW_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_CRC_LOW_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_crc_low_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_3_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_3_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_address_to_write_3_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_2_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_2_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_address_to_write_2_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_1_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_1_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_address_to_write_1_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_0_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_ADDRESS_TO_WRITE_0_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_address_to_write_0_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_CRC_CORRECTNESS_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_CRC_CORRECTNESS_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_crc_correctness_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

	fm25v02_read(2*CHANGE_BOOT_WRITE_REG, &status_reg_temp_h);
	fm25v02_read(2*CHANGE_BOOT_WRITE_REG+1, &status_reg_temp_l);
	change_boot_registers.change_boot_write_reg = ((((uint16_t)status_reg_temp_h)&0x00FF)<<8)|(((uint16_t)status_reg_temp_l)&0x00FF);

}

//-------------------------------------------------------------------------------------------------

//-----Блок расчета CRC16 для Modbus---------------------------------------------------------------

const unsigned char MB_Slave_aucCRCHi[] =
{
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

const unsigned char MB_Slave_aucCRCLo[] =
{
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
  0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
  0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
  0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
  0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
  0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
  0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
  0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
  0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

//------------------------------------------------------------------------------------------------------



//--------------------- Процедура расчета CRC16 -------------------------------------

 unsigned int CRC16( unsigned char * pucFrame, unsigned int usLen ){
  unsigned char MB_Slave_ucCRCHi = 0xFF;
  unsigned char MB_Slave_ucCRCLo = 0xFF;
  int iIndex;
  //
  while( usLen-- ){
    iIndex = MB_Slave_ucCRCLo ^ *( pucFrame++ );
    MB_Slave_ucCRCLo = MB_Slave_ucCRCHi ^ MB_Slave_aucCRCHi[iIndex];
    MB_Slave_ucCRCHi = MB_Slave_aucCRCLo[iIndex];
  }
  return MB_Slave_ucCRCHi << 8 | MB_Slave_ucCRCLo;
}

//------------------------------------------------------------------------------------

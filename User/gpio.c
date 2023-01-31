#include "stm32f4xx_hal.h"



//---функции включения и отключения светодиодов, зуммера и транзистора охранного светодиода-----------------

void BUZ_ON(void) // включение зуммера
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
}

void BUZ_OFF(void) // выключение зуммера
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
}

void LED_OUT_ON(void) // включение охранного светодиода
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

void LED_OUT_OFF(void) // выключение охранного светодиода
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

void LED_OUT_TOGGLE(void) // переключение охранного светодиода
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void LED_VD3_ON(void) // включение светодиода VD3
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void LED_VD3_OFF(void) // выключение светодиода VD3
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void LED_VD4_ON(void) // включение светодиода VD4
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);
}

void LED_VD4_OFF(void) // выключение светодиода VD4
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);
}

void LED_VD5_ON(void) // включение светодиода VD5
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
}

void LED_VD5_OFF(void) // выключение светодиода VD5
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
}

void LED_VD3_TOGGLE(void) // переключение светодиода VD3
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void LED_VD4_TOGGLE(void) // переключение светодиода VD4
{
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_8);
}

void LED_VD5_TOGGLE(void) // переключение светодиода VD5
{
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
}

void LED1_OFF(void) // выключение светодиода LED1
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
}

void LED1_ON(void) // включение светодиода LED1
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED2_OFF(void) // выключение светодиода LED2
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
}

void LED2_ON(void) // включение светодиода LED2
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);
}

void LED3_OFF(void) // выключение светодиода LED3
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void LED3_ON(void) // включение светодиода LED3
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void LED4_OFF(void) // выключение светодиода LED4
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
}

void LED4_ON(void) // включение светодиода LED4
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
}

void LED5_OFF(void) // выключение светодиода LED5
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);
}

void LED5_ON(void) // включение светодиода LED5
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
}

void LED6_OFF(void) // выключение светодиода LED6
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
}

void LED6_ON(void) // включение светодиода LED6
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
}

void LED7_OFF(void) // выключение светодиода LED7
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET);
}

void LED7_ON(void) // вклюдченище светодиода LED7
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_RESET);
}

void LED8_OFF(void) // выключение светодиода LED8
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_SET);
}

void LED8_ON(void) // включение светодиода LED8
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_RESET);
}

void LED1_TOGGLE(void) // переключение светодиода LED1
{
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_2);
}

void LED2_TOGGLE(void) // переключение светодиода LED2
{
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_3);
}

void LED3_TOGGLE(void) // переключение светодиода LED3
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

void LED4_TOGGLE(void) // переключение светодиода LED4
{
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_4);
}

void LED5_TOGGLE(void) // переключение светодиода LED5
{
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_5);
}

void LED6_TOGGLE(void) // переключение светодиода LED6
{
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_3);
}

void LED7_TOGGLE(void) // переключение светодиода LED7
{
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);
}

void LED8_TOGGLE(void) // переключение светодиода LED8
{
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_11);
}

//-------------------------------------------------------------------------------------------------------

//---функции включения и отключения фаз-----------------
void PHASE_A_ON(void)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
}

void PHASE_A_OFF(void)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
}

void PHASE_B_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

void PHASE_B_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

void PHASE_C_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void PHASE_C_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

//------------------------------------------------------




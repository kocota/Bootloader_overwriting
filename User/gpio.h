#ifndef GPIO
#define GPIO

//----прототипы функций включения и отключения светодиодов, зуммера и транзистора охранного светодиода---------
void BUZ_ON(void);
void BUZ_OFF(void);

void LED_OUT_ON(void);
void LED_OUT_OFF(void);

void LED_OUT_TOGGLE(void);

void LED_VD3_ON(void);
void LED_VD3_OFF(void);
void LED_VD4_ON(void);
void LED_VD4_OFF(void);
void LED_VD5_ON(void);
void LED_VD5_OFF(void);

void LED_VD3_TOGGLE(void);
void LED_VD4_TOGGLE(void);
void LED_VD5_TOGGLE(void);

void LED1_OFF(void);
void LED1_ON(void);
void LED2_OFF(void);
void LED2_ON(void);
void LED3_OFF(void);
void LED3_ON(void);
void LED4_OFF(void);
void LED4_ON(void);
void LED5_OFF(void);
void LED5_ON(void);
void LED6_OFF(void);
void LED6_ON(void);
void LED7_OFF(void);
void LED7_ON(void);
void LED8_OFF(void);
void LED8_ON(void);

void LED1_TOGGLE(void);
void LED2_TOGGLE(void);
void LED3_TOGGLE(void);
void LED4_TOGGLE(void);
void LED5_TOGGLE(void);
void LED6_TOGGLE(void);
void LED7_TOGGLE(void);
void LED8_TOGGLE(void);
//-------------------------------------------------------------------------------------------------------------

//---прототипы функций включения и отключения фаз-------------------
void PHASE_A_ON(void);
void PHASE_A_OFF(void);
void PHASE_B_ON(void);
void PHASE_B_OFF(void);
void PHASE_C_ON(void);
void PHASE_C_OFF(void);
//------------------------------------------------------------------



#endif

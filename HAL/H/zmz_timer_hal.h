#ifndef _ZMZ_TIMER_HAL_H
#define _ZMZ_TIMER_HAL_H

#include "zmz_system_hardware.h"

void Timer_Set_Duty_Hal(u8 timer_index, double duty_in_percentage);
void Timer_Set_Period_Limited_Hal(u8 timer_index, double Period_ms);
bool Timer_Check_Delay_Timer_Hal(u8 *timer_index);
u8 Timer_Get_Timer_Nbr_Hal(u8 timer_index);
float Timer_Get_Counting_Freq_Hal(u8 timer_index);
TIM_TypeDef *Timer_Get_Timer_Instance_Hal(u8 timer_index);
void Timer_Init_Hal(void);

#endif

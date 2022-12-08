#include "zmz_timer_hal.h"
#include "zmz_timer_drv_STM32F103.h"
#include "zmz_uart_hal.h"

void Timer_Set_Duty_Hal(u8 timer_index, double duty_in_percentage)
{
    Timer_Set_Duty_Drv(timer_index, duty_in_percentage);
}

void Timer_Set_Period_Limited_Hal(u8 timer_index, double Period_ms)
{
    Timer_Set_Period_Limited_Drv(timer_index, Period_ms);
}

bool Timer_Check_Delay_Timer_Hal(u8 *timer_index)
{
    return Timer_Check_Delay_Timer_Drv(timer_index);
}

u8 Timer_Get_Timer_Nbr_Hal(u8 timer_index)
{
    return Timer_Get_Timer_Nbr_Drv(timer_index);
}

float Timer_Get_Counting_Freq_Hal(u8 timer_index)
{
    return Timer_Get_Counting_Freq_Drv(timer_index);
}

TIM_TypeDef *Timer_Get_Timer_Instance_Hal(u8 timer_index)
{
    return Timer_Get_Timer_Instance_Drv(timer_index);
}

void Timer_Init_Hal(void)
{
    Timer_Init_Drv();
}

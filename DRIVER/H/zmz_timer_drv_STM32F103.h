#ifndef _ZMZ_TIMER_DRV_STM32F103_H
#define _ZMZ_TIMER_DRV_STM32F103_H

#include "zmz_uart_hal.h"
#include "zmz_system_hardware.h"

#define ZSS_TIMER_LOGD(KEY, format, ...) ZSS_LOGD("TIMER", KEY, format, ##__VA_ARGS__)
#define ZSS_TIMER_LOGI(format, ...) ZSS_LOGI("TIMER", format, ##__VA_ARGS__)
#define ZSS_TIMER_LOGW(format, ...) ZSS_LOGW("TIMER", format, ##__VA_ARGS__)
#define ZSS_TIMER_LOGE(format, ...) ZSS_LOGE("TIMER", format, ##__VA_ARGS__)
#define ZSS_TIMER_LOGF(format, ...) ZSS_LOGF("TIMER", format, ##__VA_ARGS__)

#define ARR_MIN 2

#define TIMER_DUTY_MIN 0
#define TIMER_DUTY_MAX 100

#define MOTOR_PRESCALER 1
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_INITIAL_DUTY TIMER_DUTY_MIN

/* better not be greater than 360 */
#define PRESCALER_4_DELAY 360

#define TIMER_SYSTEM_TIME_IDX 3

#define MILISECONDS_PER_SECOND 1000

#define TIMER_CASE_GPIO_INIT_LOG(CASE, PIN, FUMCTION)                                               \
    {                                                                                               \
        CASE_GPIO_INIT_LOG(CASE, PIN, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, FUMCTION) \
    }

typedef enum timer_type {
    DELAY_TIMER = 0,
    BASE_TIMER,
    PWM_TIMER,
} timer_type_e;

void Timer_Init_Drv(void);
void Timer_Set_Duty_Drv(u8 timer_index, double duty_in_percentage);
void Timer_Set_Period_Limited_Drv(u8 timer_index, double Period_ms);
bool Timer_Check_Delay_Timer_Drv(u8 *timer_index);
u8 Timer_Get_Timer_Nbr_Drv(u8 timer_index);
float Timer_Get_Counting_Freq_Drv(u8 timer_index);
TIM_TypeDef *Timer_Get_Timer_Instance_Drv(u8 timer_index);
double Timer_Get_System_Time_Second_Drv(void);
double Timer_Get_System_Time_Milisecond_Drv(void);

#endif

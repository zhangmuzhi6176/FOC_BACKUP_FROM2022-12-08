#include "zmz_delay.h"

#include "zmz_timer_hal.h"
#include "zmz_uart_hal.h"

#include "zmz_system_hardware.h"

typedef struct delay_dev {
    TIM_TypeDef *delay_instance;
    float timer_ticks_per_us;
    void (*delay_us_cb)(u32);
} delay_dev_t;

static delay_dev_t delay_dev = {
    .timer_ticks_per_us = 0,
};

static void _delay_us_timer(u32 us)
{
    u32 reload, total_ticks, last_tick, cur_tick, tick_counts = 0;

    reload = delay_dev.delay_instance->ARR;
    total_ticks = us * delay_dev.timer_ticks_per_us;
    last_tick = delay_dev.delay_instance->CNT;

    while (true) {
        cur_tick = delay_dev.delay_instance->CNT;
        if (cur_tick != last_tick) {
            if (cur_tick < last_tick) {
                tick_counts += last_tick - cur_tick;
            } else {
                tick_counts += reload - cur_tick + last_tick;
            }
            last_tick = cur_tick;
            if (tick_counts >= total_ticks) {
                return;
            }
        }
    }
}

static void _delay_us_systick(u32 us)
{
    u32 reload, total_ticks, last_tick, cur_tick, tick_counts = 0;

    reload = SysTick->LOAD;
    total_ticks = us * delay_dev.timer_ticks_per_us;
    last_tick = SysTick->VAL;

    while (true) {
        cur_tick = SysTick->VAL;
        if (cur_tick != last_tick) {
            if (cur_tick < last_tick) {
                tick_counts += last_tick - cur_tick;
            } else {
                tick_counts += reload - cur_tick + last_tick;
            }
            last_tick = cur_tick;
            if (tick_counts >= total_ticks) {
                return;
            }
        }
    }
}

void delay_init(void)
{
    bool is_delay_timer = false;
    u8 timer_index, timer_number = 255;

#ifndef USE_SYSTICK_DELAY
    is_delay_timer = Timer_Check_Delay_Timer_Hal(&timer_index);
#endif

    if (is_delay_timer) {
        timer_number = Timer_Get_Timer_Nbr_Hal(timer_index);
        delay_dev.timer_ticks_per_us = Timer_Get_Counting_Freq_Hal(timer_index) / US_PER_S;
        delay_dev.delay_instance = Timer_Get_Timer_Instance_Hal(timer_index);
        delay_dev.delay_us_cb = _delay_us_timer;
        ZSS_DELAY_LOGI("[TIM%d] is initialized for delay, timer_ticks_per_us: [%.3f].\r\n", timer_number, delay_dev.timer_ticks_per_us);
    } else {
        delay_dev.timer_ticks_per_us = Stm32_Get_Clock_MHz();
        delay_dev.delay_us_cb = _delay_us_systick;
        HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
        ZSS_DELAY_LOGI("[Systick] is used for delay.\r\n");
    }
}

void delay_us(u32 us)
{
    delay_dev.delay_us_cb(us);
}

void delay_ms(u16 ms)
{
    u32 i;
    for (i = 0; i < ms; i++)
        delay_us(1000);
}

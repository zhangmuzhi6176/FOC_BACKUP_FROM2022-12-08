#include "zmz_foc.h"
#include "zmz_mt6701.h"
#include "zmz_timer_drv_STM32F103.h"
#include "zmz_gpio_drv_STM32F103.h"
#include "zmz_system_hardware.h"

typedef struct pwm {
    TIM_OC_InitTypeDef timer_output_compare_config;

    gpio_spec_t pwm_io;
    u32 channel;
    u8 channel_index;

    double duty_in_percentage;
} pwm_t;

typedef struct timer {
    TIM_TypeDef *instance;
    u8 timer_number;
    TIM_HandleTypeDef tim_handler;

    double timer_event_freq;
    u16 prescaler;
    u16 arr;
    timer_type_e timer_type;

    IRQn_Type irq_no;
    u32 preempt_priority;
    u32 sub_priority;
    bool enable_irq;

    pwm_t pwm;

    u32 system_time_rounds;
    u16 system_time_ticks;
} timer_t;

static timer_t timer_dev_g[] = {
    [0] = {
        .instance = TIM3,
        .timer_number = 3,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = A,
                .gpio_num = 7,
            },
            .channel = TIM_CHANNEL_2,
            .channel_index = 2,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
    [1] = {
        .instance = TIM3,
        .timer_number = 3,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = A,
                .gpio_num = 6,
            },
            .channel = TIM_CHANNEL_1,
            .channel_index = 1,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
    [2] = {
        .instance = TIM3,
        .timer_number = 3,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = B,
                .gpio_num = 0,
            },
            .channel = TIM_CHANNEL_3,
            .channel_index = 3,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
    [3] = {
        .instance = TIM1,
        .timer_number = 1,
        .prescaler = PRESCALER_4_DELAY,
        .timer_type = DELAY_TIMER,
        .enable_irq = true,
        .irq_no = TIM1_UP_IRQn,
        .preempt_priority = 3,
        .sub_priority = 0,
    },
    [4] = {
        .instance = TIM3,
        .timer_number = 3,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = B,
                .gpio_num = 1,
            },
            .channel = TIM_CHANNEL_4,
            .channel_index = 4,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
    [5] = {
        .instance = TIM2,
        .timer_number = 2,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = B,
                .gpio_num = 10,
            },
            .channel = TIM_CHANNEL_3,
            .channel_index = 3,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
    [6] = {
        .instance = TIM2,
        .timer_number = 2,
        .timer_event_freq = MOTOR_PWM_FREQ,
        .timer_type = PWM_TIMER,
        .prescaler = MOTOR_PRESCALER,
        .pwm = {
            .pwm_io = {
                .gpio_grp = B,
                .gpio_num = 11,
            },
            .channel = TIM_CHANNEL_4,
            .channel_index = 4,
            .duty_in_percentage = MOTOR_PWM_INITIAL_DUTY,
        },
    },
};

static void _Timer_PWM_Init(timer_t timer_dev)
{
    timer_dev.pwm.timer_output_compare_config.OCMode = TIM_OCMODE_PWM1;
    timer_dev.pwm.timer_output_compare_config.Pulse = timer_dev.arr * (timer_dev.pwm.duty_in_percentage / 100.0);
    timer_dev.pwm.timer_output_compare_config.OCPolarity = TIM_OCPOLARITY_HIGH;

    HAL_TIM_PWM_Init(&(timer_dev.tim_handler));
    HAL_TIM_PWM_ConfigChannel(&(timer_dev.tim_handler), &(timer_dev.pwm.timer_output_compare_config), timer_dev.pwm.channel);

    HAL_TIM_PWM_Start(&(timer_dev.tim_handler), timer_dev.pwm.channel);
}

/*
    @description:
        counter_counting_freq = MAIN_FREQ / prescaler (Hz)
        timer_event_freq = counter_counting_freq / arr (Hz)
 */
void Timer_Init_Drv(void)
{
    double counter_counting_freq = 0;

    for (u32 i = 0; i < ZSS_ARRAY_SIZE(timer_dev_g); i++) {
        if (!timer_dev_g[i].prescaler) {
            /* user do not want to point prescaler */
            if (100000 < timer_dev_g[i].timer_event_freq && timer_dev_g[i].timer_event_freq <= 1000000) {
                /* counter_counting_freq = 1,000,000Hz */
                timer_dev_g[i].prescaler = Stm32_Get_Clock_MHz();
            } else if (2 < timer_dev_g[i].timer_event_freq && timer_dev_g[i].timer_event_freq <= 100000) {
                /* counter_counting_freq = 100,000Hz */
                timer_dev_g[i].prescaler = 10 * Stm32_Get_Clock_MHz();
            } else {
                /* counter_counting_freq = 10,000Hz */
                timer_dev_g[i].prescaler = 100 * Stm32_Get_Clock_MHz();
            }
        }

        counter_counting_freq = (1000000 * Stm32_Get_Clock_MHz()) / timer_dev_g[i].prescaler;
        timer_dev_g[i].arr = (u16)(counter_counting_freq / timer_dev_g[i].timer_event_freq);
        if (!timer_dev_g[i].arr) {
            timer_dev_g[i].arr = ARR_MIN;
        }

        timer_dev_g[i].tim_handler.Instance = timer_dev_g[i].instance;
        timer_dev_g[i].tim_handler.Init.Prescaler = timer_dev_g[i].prescaler - 1;
        timer_dev_g[i].tim_handler.Init.Period = timer_dev_g[i].arr - 1;
        timer_dev_g[i].tim_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        if (BASE_TIMER == timer_dev_g[i].timer_type) {
            timer_dev_g[i].tim_handler.Init.CounterMode = TIM_COUNTERMODE_UP;
            HAL_TIM_Base_Init(&(timer_dev_g[i].tim_handler));
            HAL_TIM_Base_Start_IT(&(timer_dev_g[i].tim_handler));
            ZSS_TIMER_LOGI("TIMER[%d] is initialized as BASE_TIMER at [%.3f] Hz, counter_counting_freq: [%.3f] Hz, arr: [%d].\r\n", 
            timer_dev_g[i].timer_number, timer_dev_g[i].timer_event_freq, counter_counting_freq, timer_dev_g[i].arr);
        } else if (PWM_TIMER == timer_dev_g[i].timer_type) {
            timer_dev_g[i].tim_handler.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
            _Timer_PWM_Init(timer_dev_g[i]);
            ZSS_TIMER_LOGI("TIMER[%d] CHANNEL[%d] is initialized as PWM_TIMER at [%.3f] Hz, counter_counting_freq: [%.3f] Hz, arr: [%d]\r\n",
                           timer_dev_g[i].timer_number, timer_dev_g[i].pwm.channel_index, timer_dev_g[i].timer_event_freq, counter_counting_freq, timer_dev_g[i].arr);
        } else if (DELAY_TIMER == timer_dev_g[i].timer_type) {
            timer_dev_g[i].tim_handler.Init.CounterMode = TIM_COUNTERMODE_DOWN;
            timer_dev_g[i].arr = (u16)(~0) - 1;
            HAL_TIM_Base_Init(&(timer_dev_g[i].tim_handler));
            __HAL_TIM_ENABLE(&(timer_dev_g[i].tim_handler));

            timer_dev_g[i].system_time_rounds = 0;
            timer_dev_g[i].timer_event_freq = counter_counting_freq / 0xFFFF;

            if (true == timer_dev_g[i].enable_irq) {
                HAL_TIM_Base_Start_IT(&(timer_dev_g[i].tim_handler));
            }
            ZSS_TIMER_LOGI("TIMER[%d] is initialized as DELAY_TIMER, counter_counting_freq: [%.3f] Hz, timer_event_freq: [%.3f] Hz, arr: [%d], enable_irq[%d].\r\n", 
            timer_dev_g[i].timer_number, counter_counting_freq, timer_dev_g[i].timer_event_freq, timer_dev_g[i].arr, timer_dev_g[i].enable_irq);
        }
    }

    ZSS_TIMER_LOGI("All timers are initialized\r\n");
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(timer_dev_g); i++) {
        if (htim->Instance == timer_dev_g[i].instance) {
            switch (timer_dev_g[i].timer_number) {
            case 1:
                __HAL_RCC_TIM1_CLK_ENABLE();
                break;
            case 2:
                __HAL_RCC_TIM2_CLK_ENABLE();
                break;
            case 3:
                __HAL_RCC_TIM3_CLK_ENABLE();
                break;
            case 4:
                __HAL_RCC_TIM4_CLK_ENABLE();
                break;
            default:
                ZSS_ASSERT_WITH_LOG("__HAL_RCC_TIM[%d]_CLK_ENABLE Unsupported", timer_dev_g[i].timer_number);
            }

            if (true == timer_dev_g[i].enable_irq) {
                HAL_NVIC_SetPriority(timer_dev_g[i].irq_no, timer_dev_g[i].preempt_priority, timer_dev_g[i].sub_priority);
                HAL_NVIC_EnableIRQ(timer_dev_g[i].irq_no);
                ZSS_TIMER_LOGI("TIMER[%d] is set as preempt_priority:[%d], sub_priority:[%d]\r\n",
                            timer_dev_g[i].timer_number, timer_dev_g[i].preempt_priority, timer_dev_g[i].sub_priority);
            }
        }
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(timer_dev_g); i++) {
        if (htim->Instance == timer_dev_g[i].instance) {
            switch (timer_dev_g[i].timer_number) {
            case 1:
                __HAL_RCC_TIM1_CLK_ENABLE();
                break;
            case 2:
                __HAL_RCC_TIM2_CLK_ENABLE();
                __HAL_AFIO_REMAP_TIM2_ENABLE();
                /* __HAL_AFIO_REMAP_SWJ_NOJTAG(); */
                break;
            case 3:
                __HAL_RCC_TIM3_CLK_ENABLE();
                break;
            case 4:
                __HAL_RCC_TIM4_CLK_ENABLE();
                break;
            default:
                ZSS_ASSERT_WITH_LOG("__HAL_RCC_TIM[%d]_CLK_ENABLE Unsupported", timer_dev_g[i].timer_number);
            }

            switch (timer_dev_g[i].pwm.pwm_io.gpio_grp) {
                TIMER_CASE_GPIO_INIT_LOG(A, timer_dev_g[i].pwm.pwm_io.gpio_num, "PWM_TIMER");
                TIMER_CASE_GPIO_INIT_LOG(B, timer_dev_g[i].pwm.pwm_io.gpio_num, "PWM_TIMER");
                TIMER_CASE_GPIO_INIT_LOG(C, timer_dev_g[i].pwm.pwm_io.gpio_num, "PWM_TIMER");
                TIMER_CASE_GPIO_INIT_LOG(D, timer_dev_g[i].pwm.pwm_io.gpio_num, "PWM_TIMER");
                TIMER_CASE_GPIO_INIT_LOG(E, timer_dev_g[i].pwm.pwm_io.gpio_num, "PWM_TIMER");
            }
        }
    }
}

void Timer_Set_Duty_Drv(u8 timer_index, double duty_in_percentage)
{
    u16 compare = timer_dev_g[timer_index].arr * duty_in_percentage / 100;

    timer_dev_g[timer_index].pwm.duty_in_percentage = duty_in_percentage;
    *((__IO uint32_t *)(&(timer_dev_g[timer_index].tim_handler.Instance->CCR1)) + (u32)(timer_dev_g[timer_index].pwm.channel_index - 1)) = compare;
}

/* timer_dev_g[timer_index].prescaler is not calculated in this function to save time, that's why it's called _limited */
void Timer_Set_Period_Limited_Drv(u8 timer_index, double Period_ms)
{
    double counter_counting_freq = (1000000 * Stm32_Get_Clock_MHz()) / timer_dev_g[timer_index].prescaler;

    timer_dev_g[timer_index].timer_event_freq = 1000.0/Period_ms;
    timer_dev_g[timer_index].arr = (u16)(counter_counting_freq / timer_dev_g[timer_index].timer_event_freq);

    if (!timer_dev_g[timer_index].arr) {
        timer_dev_g[timer_index].arr = ARR_MIN;
    }

    *((__IO uint32_t *)(&(timer_dev_g[timer_index].tim_handler.Instance->ARR))) = timer_dev_g[timer_index].arr;
}

bool Timer_Check_Delay_Timer_Drv(u8 *timer_index)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(timer_dev_g); i++) {
        if (DELAY_TIMER == timer_dev_g[i].timer_type) {
            *timer_index = i;
            return true;
        }
    }
    return false;
}

u8 Timer_Get_Timer_Nbr_Drv(u8 timer_index)
{
    return timer_dev_g[timer_index].timer_number;
}

float Timer_Get_Counting_Freq_Drv(u8 timer_index)
{
    float counter_counting_freq = (float)(1000000 * Stm32_Get_Clock_MHz()) / (float)timer_dev_g[timer_index].prescaler;
    return counter_counting_freq;
}

TIM_TypeDef *Timer_Get_Timer_Instance_Drv(u8 timer_index)
{
    return timer_dev_g[timer_index].instance;
}

static u32 _Timer_Get_System_Time_Rounds(void)
{
    if(timer_dev_g[TIMER_SYSTEM_TIME_IDX].system_time_rounds >= 0xFFFF) {
        timer_dev_g[TIMER_SYSTEM_TIME_IDX].system_time_rounds = 0;
    }
    return timer_dev_g[TIMER_SYSTEM_TIME_IDX].system_time_rounds;
}

static u16 _Timer_Get_System_Time_Ticks(void)
{
    return (0xFFFF - timer_dev_g[TIMER_SYSTEM_TIME_IDX].tim_handler.Instance->CNT);
}

double Timer_Get_System_Time_Second_Drv(void)
{
    u32 rounds_from_boot = 0;
    u16 current_ticks = 0;
    double now_second = 0;

    rounds_from_boot = _Timer_Get_System_Time_Rounds();
    current_ticks = _Timer_Get_System_Time_Ticks();

    now_second = (rounds_from_boot + ((double)current_ticks / 0xFFFF)) * (1 / timer_dev_g[TIMER_SYSTEM_TIME_IDX].timer_event_freq);

    return now_second;
}

double Timer_Get_System_Time_Milisecond_Drv(void)
{
    return Timer_Get_System_Time_Second_Drv() * MILISECONDS_PER_SECOND;
}

/* --------------------------------------------------------------- Timer Interrupt callbacks --------------------------------------------------------------- */

/* TODO:在上层实现
一系列对BLDC参数调整的API（方向，转速，力矩，目标位置）
和void BLDC_Quantum_Ops(void)
在TIM2中断回调中调用BLDC_Quatum_Ops()
 */

void TIM1_UP_IRQHandler(void)
{
    // Clear IT
    timer_dev_g[TIMER_SYSTEM_TIME_IDX].tim_handler.Instance->SR = ~(TIM_IT_UPDATE);
    timer_dev_g[TIMER_SYSTEM_TIME_IDX].system_time_rounds++;
}

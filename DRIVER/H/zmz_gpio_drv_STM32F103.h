#ifndef _ZMZ_GPIO_DRV_STM32F103_H
#define _ZMZ_GPIO_DRV_STM32F103_H

#include "zmz_uart_hal.h"
#include "zmz_gpio_drv_STM32F103.h"
#include "zmz_system_hardware.h"
#include "stm32f103xb.h"

#define ZSS_GPIO_LOGD(KEY, format, ...) ZSS_LOGD("GPIO", KEY, format, ##__VA_ARGS__)
#define ZSS_GPIO_LOGI(format, ...) ZSS_LOGI("GPIO", format, ##__VA_ARGS__)
#define ZSS_GPIO_LOGW(format, ...) ZSS_LOGW("GPIO", format, ##__VA_ARGS__)
#define ZSS_GPIO_LOGE(format, ...) ZSS_LOGE("GPIO", format, ##__VA_ARGS__)
#define ZSS_GPIO_LOGF(format, ...) ZSS_LOGF("GPIO", format, ##__VA_ARGS__)

typedef enum io_group {
    A = 1,
    B,
    C,
    D,
    E,
    F,
    G,
} io_group_e;

typedef struct gpio_spec {
    io_group_e gpio_grp;
    u8 gpio_num;
} gpio_spec_t;

#define GPIO_INIT(GPIO, PIN, MODE, PULL, SPEED) \
    {                                           \
        __HAL_RCC_##GPIO##_CLK_ENABLE();        \
        GPIO_InitTypeDef GPIO_Init;             \
        GPIO_Init.Pin = (u32)1 << PIN;          \
        GPIO_Init.Mode = MODE;                  \
        GPIO_Init.Pull = PULL;                  \
        GPIO_Init.Speed = SPEED;                \
        HAL_GPIO_Init(GPIO, &GPIO_Init);        \
    }

#define CASE_GPIO_INIT_LOG(GPIO_INDEX, PIN, MODE, PULL, SPEED, FUMCTION)                           \
    {                                                                                              \
    case GPIO_INDEX:                                                                               \
        GPIO_INIT(GPIO##GPIO_INDEX##, PIN, MODE, PULL, SPEED);                                     \
        ZSS_GPIO_LOGI("GPIO%s[%d] is initialized as %s.\r\n", ZSS_STR(GPIO_INDEX), PIN, FUMCTION); \
        break;                                                                                     \
    }

#define CASE_GPIO_INIT(GPIO_INDEX, PIN, MODE, PULL, SPEED)     \
    {                                                          \
    case GPIO_INDEX:                                           \
        GPIO_INIT(GPIO##GPIO_INDEX##, PIN, MODE, PULL, SPEED); \
        break;                                                 \
    }

#endif

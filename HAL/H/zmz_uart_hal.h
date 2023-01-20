#ifndef _ZMZ_UART_HAL_H
#define _ZMZ_UART_HAL_H
#include "zmz_delay.h"
#include "zmz_led.h"
#include "zmz_uart_drv_STM32F103.h"
#include "zmz_system_hardware.h"
#include "string.h"

#define FILE_NAME_LEN_MAX 35
#define MODULE_NAME_LEN_MAX 10
#define LEVEL_NAME_LEN_MAX 1
#define NO_PATH_FILE_NAME ((NULL == strstr(__FILE__, "\\")) ? (__FILE__) : (strrchr(__FILE__, '\\') + 1))

#define ZSS_LOG(LOG_LEVEL, MODULE, format, ...)                                                                                            \
    {                                                                                                                                      \
        printf("FILE:%-*s  |  LINE:[%4d]  |  %-*s  |  %-*s  |  " format,                                                                   \
               FILE_NAME_LEN_MAX, NO_PATH_FILE_NAME, __LINE__, LEVEL_NAME_LEN_MAX, LOG_LEVEL, MODULE_NAME_LEN_MAX, MODULE, ##__VA_ARGS__); \
    }

#define ZSS_ASSERT_WITH_LOG(format, ...)                                                                                               \
    {                                                                                                                                  \
        printf("FILE:%-*s  |  LINE:[%4d]  |  %-*s  |  %-*s  |  " format,                                                               \
               FILE_NAME_LEN_MAX, NO_PATH_FILE_NAME, __LINE__, LEVEL_NAME_LEN_MAX, "F", MODULE_NAME_LEN_MAX, "ASSERT", ##__VA_ARGS__); \
        while (1)                                                                                                                      \
        {                                                                                                                              \
            RGB_Led_Blink(RGB_LED_I, RGB_LED_PURPLE, 1, 600);                                                                          \
        }                                                                                                                              \
    }

#define ZSS_LOGD(MODULE, KEY, format, ...)                                      \
    {                                                                           \
        if (Get_Uart_LOG_D_Status(UART_1) && Match_Uart_LOG_D_KEY(UART_1, KEY)) \
        {                                                                       \
            ZSS_LOG("D", MODULE, format, ##__VA_ARGS__)                         \
        }                                                                       \
    }
#define ZSS_LOGI(MODULE, format, ...) ZSS_LOG("I", MODULE, format, ##__VA_ARGS__)
#define ZSS_LOGW(MODULE, format, ...) ZSS_LOG("W", MODULE, format, ##__VA_ARGS__)
#define ZSS_LOGE(MODULE, format, ...) ZSS_LOG("E", MODULE, format, ##__VA_ARGS__)
#define ZSS_LOGF(MODULE, format, ...) ZSS_LOG("F", MODULE, format, ##__VA_ARGS__)
#define ZSS_LOGPLOT printf

void Uart_Cmd(u8 uart_index, const char *cmd);

#endif

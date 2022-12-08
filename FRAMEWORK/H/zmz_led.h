#ifndef _ZMZ_LED_H
#define _ZMZ_LED_H

#include "zmz_uart_hal.h"
#include "zmz_timer_drv_STM32F103.h"
#include "zmz_system_hardware.h"

#define ZSS_LED_LOGD(KEY, format, ...) ZSS_LOGD("LED", KEY, format, ##__VA_ARGS__)
#define ZSS_LED_LOGI(format, ...) ZSS_LOGI("LED", format, ##__VA_ARGS__)
#define ZSS_LED_LOGW(format, ...) ZSS_LOGW("LED", format, ##__VA_ARGS__)
#define ZSS_LED_LOGE(format, ...) ZSS_LOGE("LED", format, ##__VA_ARGS__)
#define ZSS_LED_LOGF(format, ...) ZSS_LOGF("LED", format, ##__VA_ARGS__)

#define LED_DUTY_MIN TIMER_DUTY_MIN
#define LED_DUTY_MAX TIMER_DUTY_MAX

typedef enum rgb_led_index {
    RGB_LED_I = 0,
} rgb_led_index_e;

typedef enum rgb_led_channel {
    RGB_LED_CHANNEL_R = 0,
    RGB_LED_CHANNEL_G,
    RGB_LED_CHANNEL_B,
    RGB_LED_CHANNEL_NUM
} rgb_led_channel_e;

typedef enum rgb_led_color_spec {
    RGB_LED_RED = 0,
    RGB_LED_GREEN,
    RGB_LED_BLUE,
    RGB_LED_LAKE_BLUE,
    RGB_LED_PURPLE,
    RGB_LED_MAGENTA,
    RGB_LED_ORANGE,
    RGB_LED_YELLOW,
    RGB_LED_COLOR_NUM
} rgb_led_color_spec_e;

void RGB_Led_Set(rgb_led_index_e rgb_led_index, double r, double g, double b);
/* param @ intensity: [0, 1] */
void RGB_Led_Set_Color(rgb_led_index_e rgb_led_index, rgb_led_color_spec_e color, double intensity);
void RGB_Led_Init(void);

#endif

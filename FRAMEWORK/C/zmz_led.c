#include "zmz_led.h"
#include "zmz_timer_hal.h"

typedef struct rgb_led_dev {
    u8 rgb_led_channel[RGB_LED_CHANNEL_NUM];
} rgb_led_dev_t;

typedef struct rgb_led_color {
    double R;
    double G;
    double B;
} rgb_led_color_t;

rgb_led_dev_t rgb_led_dev_g[] = {
    [RGB_LED_I] = {
        .rgb_led_channel[RGB_LED_CHANNEL_R] = 4,
        .rgb_led_channel[RGB_LED_CHANNEL_G] = 5,
        .rgb_led_channel[RGB_LED_CHANNEL_B] = 6,
    },
};

rgb_led_color_t rgb_led_color_g[] = {
    [RGB_LED_RED] = {100, 0, 0},
    [RGB_LED_GREEN] = {0, 100, 0},
    [RGB_LED_BLUE] = {0, 0, 100},
    [RGB_LED_LAKE_BLUE] = {10, 46, 100},
    [RGB_LED_PURPLE] = {30, 0, 100},
    [RGB_LED_MAGENTA] = {50, 0, 100},
    [RGB_LED_ORANGE] = {90, 50, 0},
    [RGB_LED_YELLOW] = {70, 100, 0},
};

void RGB_Led_Set(rgb_led_index_e rgb_led_index, double r, double g, double b)
{
    Timer_Set_Duty_Hal(rgb_led_dev_g[rgb_led_index].rgb_led_channel[RGB_LED_CHANNEL_R], (double)(LED_DUTY_MAX - r));
    Timer_Set_Duty_Hal(rgb_led_dev_g[rgb_led_index].rgb_led_channel[RGB_LED_CHANNEL_G], (double)(LED_DUTY_MAX - g));
    Timer_Set_Duty_Hal(rgb_led_dev_g[rgb_led_index].rgb_led_channel[RGB_LED_CHANNEL_B], (double)(LED_DUTY_MAX - b));
}

void RGB_Led_Set_Color(rgb_led_index_e rgb_led_index, rgb_led_color_spec_e color, double intensity)
{
    RGB_Led_Set(rgb_led_index, (rgb_led_color_g[color].R * intensity), (rgb_led_color_g[color].G * intensity), (rgb_led_color_g[color].B * intensity));
}

void RGB_Led_Blink(rgb_led_index_e rgb_led_index, rgb_led_color_spec_e color, double intensity, double period_ms)
{
    RGB_Led_Set_Color(rgb_led_index, color, intensity);
    delay_ms(period_ms / 2);
    RGB_Led_Set_Color(rgb_led_index, color, 0);
    delay_ms(period_ms / 2);
}

void RGB_Led_Blink_Times(rgb_led_index_e rgb_led_index, rgb_led_color_spec_e color, double intensity, u8 Times)
{
    for (u8 i = 0; i < Times; i++) {
        RGB_Led_Blink(rgb_led_index, color, intensity, 300);
    }
}

void RGB_Led_Init(void)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(rgb_led_dev_g); i++) {
        RGB_Led_Set((rgb_led_index_e)i, TIMER_DUTY_MIN, TIMER_DUTY_MIN, TIMER_DUTY_MIN);
        ZSS_LED_LOGI("RGB_LED [%d] is initialized.\r\n", i);
    }
}

#ifndef _ZMZ_DELAY_H
#define _ZMZ_DELAY_H
#include "zmz_system_hardware.h"

/* #define USE_SYSTICK_DELAY */

#define ZSS_DELAY_LOGD(KEY, format, ...) ZSS_LOGD("DELAY", KEY, format, ##__VA_ARGS__)
#define ZSS_DELAY_LOGI(format, ...) ZSS_LOGI("DELAY", format, ##__VA_ARGS__)
#define ZSS_DELAY_LOGW(format, ...) ZSS_LOGW("DELAY", format, ##__VA_ARGS__)
#define ZSS_DELAY_LOGE(format, ...) ZSS_LOGE("DELAY", format, ##__VA_ARGS__)
#define ZSS_DELAY_LOGF(format, ...) ZSS_LOGF("DELAY", format, ##__VA_ARGS__)

#define US_PER_S 1000000U

void delay_init(void);
void delay_ms(u16 ms);
void delay_us(u32 us);

#endif

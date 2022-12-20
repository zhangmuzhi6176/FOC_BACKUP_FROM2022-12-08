#ifndef _ZMZ_FILTER_H
#define _ZMZ_FILTER_H

#include "zmz_uart_hal.h"
#include "zmz_system_hardware.h"

#define ZSS_FILTER_LOGD(KEY, format, ...) ZSS_LOGD("FILTER", KEY, format, ##__VA_ARGS__)
#define ZSS_FILTER_LOGI(format, ...) ZSS_LOGI("FILTER", format, ##__VA_ARGS__)
#define ZSS_FILTER_LOGW(format, ...) ZSS_LOGW("FILTER", format, ##__VA_ARGS__)
#define ZSS_FILTER_LOGE(format, ...) ZSS_LOGE("FILTER", format, ##__VA_ARGS__)
#define ZSS_FILTER_LOGF(format, ...) ZSS_LOGF("FILTER", format, ##__VA_ARGS__)

typedef struct filter_moving_average {
    double window[10];
    double sum;
    u8 window_size;
    u8 index;
} filter_moving_average_t;

typedef struct filter_1st_lp {
    double val_last;
    double alpha;
} filter_1st_lp_t;

double Filter_Moving_Average(filter_moving_average_t *filter, double val);
double Filter_1st_LP(filter_1st_lp_t *filter, double val);

#endif

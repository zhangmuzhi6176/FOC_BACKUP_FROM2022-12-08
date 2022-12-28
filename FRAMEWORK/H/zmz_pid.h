#ifndef _ZMZ_PID_H
#define _ZMZ_PID_H

#include "zmz_uart_hal.h"
#include "zmz_system_hardware.h"

#define ZSS_PID_LOGD(KEY, format, ...) ZSS_LOGD("PID", KEY, format, ##__VA_ARGS__)
#define ZSS_PID_LOGI(format, ...) ZSS_LOGI("PID", format, ##__VA_ARGS__)
#define ZSS_PID_LOGW(format, ...) ZSS_LOGW("PID", format, ##__VA_ARGS__)
#define ZSS_PID_LOGE(format, ...) ZSS_LOGE("PID", format, ##__VA_ARGS__)
#define ZSS_PID_LOGF(format, ...) ZSS_LOGF("PID", format, ##__VA_ARGS__)

typedef struct PID_param {
    double P;
    double I;
    double D;
    double error_accum;
    double error_accum_max;
    double error_accum_threshold;
    double last_error;
    double last_result;
} PID_param_t;

PID_param_t *PID_Init_Param(double P, double I, double D);
void PID_Init(PID_param_t *pid);
double PID_calc_Pos(PID_param_t *param, double ref_val, double cur_val);
double PID_calc_Inc(PID_param_t *param, double ref_val, double cur_val);

#endif

#ifndef _ZMZ_FOC_H
#define _ZMZ_FOC_H

#include "zmz_uart_hal.h"
#include "zmz_timer_drv_STM32F103.h"
#include "zmz_system_hardware.h"

#define ZSS_FOC_LOGD(KEY, format, ...) ZSS_LOGD("FOC", KEY, format, ##__VA_ARGS__)
#define ZSS_FOC_LOGI(format, ...) ZSS_LOGI("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGW(format, ...) ZSS_LOGW("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGE(format, ...) ZSS_LOGE("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGF(format, ...) ZSS_LOGF("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGPLOT ZSS_LOGPLOT

#define ZSS_FOC_LOGPLOT_TRIPPLE(a, b, c, mag) {ZSS_FOC_LOGPLOT("%-*.6f, %-*.6f, %-*.6f\r\n", FOC_PLOT_WIDTH, a * mag, FOC_PLOT_WIDTH, b * mag, FOC_PLOT_WIDTH, c * mag);}

#define BLDC_ZERO_TORQUE TIMER_DUTY_MIN
#define BLDC_MAX_TORQUE TIMER_DUTY_MAX

#define BLDC_CURRENT_CALI_DIFF_THRESH 300

#define FOC_PLOT_WIDTH 15

typedef struct foc_current {
    double I_a;
    double I_b;
    double I_c;
    double I_alpha;
    double I_beta;
    double I_d;
    double I_q;
} foc_current_t;

typedef enum foc_index {
    FOC_I = 0,
} foc_index_e;

typedef enum foc_pid {
    FOC_PID_Q = 0,
    FOC_PID_D,
    FOC_PID_SPEED,
    FOC_PID_POSITION,
    FOC_PID_NUM
} foc_pid_e;

typedef enum foc_filter_bind {
    FOC_FILTER_BIND_TORQ = 0,
    FOC_FILTER_BIND_SPEED,
    FOC_FILTER_BIND_POSITION,
    FOC_FILTER_BIND_NUM
} foc_filter_bind_e;

typedef enum bldc_phase {
    PHASE_A = 0,
    PHASE_B,
    PHASE_C,
    PHASE_NUM
} bldc_phase_e;

typedef enum svpwm_enable {
    SVPWM_ENABLE = 0,
    SVPWM_DISABLE,
} svpwm_enable_e;

typedef enum svpwm_phase {
    SVPWM_PHASE_INITIAL = -1,
    SVPWM_PHASE_I = 0,
    SVPWM_PHASE_II,
    SVPWM_PHASE_III,
    SVPWM_PHASE_IV,
    SVPWM_PHASE_V,
    SVPWM_PHASE_VI,
    SVPWM_PHASE_NUM,
} svpwm_phase_e;

typedef enum bldc_direction {
    BLDC_STOP = 0,
    BLDC_CW, 
    BLDC_CCW,
} bldc_direction_e;

static char *svpwm_phase_e2s[] = {
    [SVPWM_PHASE_I] = "SVPWM_PHASE_I",
    [SVPWM_PHASE_II] = "SVPWM_PHASE_II",
    [SVPWM_PHASE_III] = "SVPWM_PHASE_III",
    [SVPWM_PHASE_IV] = "SVPWM_PHASE_IV",
    [SVPWM_PHASE_V] = "SVPWM_PHASE_V",
    [SVPWM_PHASE_VI] = "SVPWM_PHASE_VI",
};

void SVPWM_Generate_Elec_Ang(foc_index_e index, double elec_angle_deg, double svpwm_duty);
void SVPWM_Generate_Mech_Ang(foc_index_e index, double mech_angle_deg, double svpwm_duty);
void SVPWM_Generate_Alpha_Beta(foc_index_e index, foc_current_t current);
void SVPWM_Disable(foc_index_e index);
void SVPWM_Enable(foc_index_e index);
void FOC_Current_Plot(foc_index_e index);

void FOC_Current_Print(foc_index_e index);
void FOC_Init(void);

/* 
    param @ Q_ref: [-1, 1]
 */
void FOC_Keep_Torque(foc_index_e index, double Q_ref);

/* 
    param @ speed_ref: [-2160, 2160]
 */
void FOC_Keep_Speed(foc_index_e index, double speed_ref);

/* 
    param @ ref_mech_angle_deg: [0, 360)
    param @ intensity: [0, 1]
 */
void FOC_Keep_Position(foc_index_e index, double ref_mech_angle_deg, double intensity);

#endif

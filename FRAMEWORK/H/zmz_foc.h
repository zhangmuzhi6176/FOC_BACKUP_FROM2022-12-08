#ifndef _ZMZ_FOC_H
#define _ZMZ_FOC_H

#include "zmz_filter.h"
#include "zmz_led.h"
#include "zmz_mt6701.h"
#include "zmz_pid.h"

#include "zmz_uart_hal.h"
#include "zmz_timer_drv_STM32F103.h"
#include "zmz_adc_drv_STM32F103.h"
#include "zmz_system_hardware.h"

#define ZSS_FOC_LOGD(KEY, format, ...) ZSS_LOGD("FOC", KEY, format, ##__VA_ARGS__)
#define ZSS_FOC_LOGI(format, ...) ZSS_LOGI("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGW(format, ...) ZSS_LOGW("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGE(format, ...) ZSS_LOGE("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGF(format, ...) ZSS_LOGF("FOC", format, ##__VA_ARGS__)
#define ZSS_FOC_LOGPLOT ZSS_LOGPLOT

#define ZSS_FOC_LOGPLOT_TRIPPLE(a, b, c, mag) {ZSS_FOC_LOGPLOT("%-*.5f, %-*.5f, %-*.5f\r\n", FOC_PLOT_WIDTH, a * mag, FOC_PLOT_WIDTH, b * mag, FOC_PLOT_WIDTH, c * mag);}

#define BLDC_ZERO_TORQUE                    TIMER_DUTY_MIN
#define BLDC_MAX_TORQUE                     TIMER_DUTY_MAX

#define BLDC_CURRENT_CALI_DIFF_THRESH       300

#define FOC_PLOT_WIDTH                      10
#define FOC_NAME_MAX                        30

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

typedef struct svpwm_stage {
    double stage_short;
    double stage_middle;
    double stage_long;
} svpwm_stage_t;

typedef struct oscilation_detect {
    u8 sign_change_cnt_thresh;
    u16 sign_unchange_cnt_thresh;

    double subject_last;
    u16 sign_unchanged_count;
    u32 oscilation_count;
} oscilation_detect_t;

typedef struct foc_dev {
    char                        name[FOC_NAME_MAX];

    u8                          bldc_tim_idx[PHASE_NUM];
    u8                          encoder_idx;
    u8                          cali_pole_num;

    svpwm_enable_e              svpwm_enable;

    double                      cali_Magfield_2_Enc_angle_offset;
    double                      mech_angle_current;

    bool                        cali_max_cur;
    u8                          current_adc_chnl[PHASE_NUM];
    double                      current_adc_offset[PHASE_NUM]; /* Calibration required */
    double                      current_max[PHASE_NUM];        /* Calibration required */
    double                      current_min[PHASE_NUM];        /* Calibration required */
    u8                          current_adc_idx;

    PID_param_t                 foc_pid[FOC_PID_NUM];
    filter_bind_t               filter_bind_param[FOC_FILTER_BIND_NUM];

    double                      Q_offset;
    double                      Q_ref_last;
    double                      Torq_I_val_l;
    double                      Torq_I_val_h;

    double                      time_last;
    double                      mech_angle_last;

    oscilation_detect_t         position_oscilation_detector;
} foc_dev_t;

void SVPWM_Generate_Elec_Ang(foc_index_e index, double elec_angle_deg, double svpwm_duty);
void SVPWM_Generate_Mech_Ang(foc_index_e index, double mech_angle_deg, double svpwm_duty);
void SVPWM_Generate_Alpha_Beta(foc_index_e index, foc_current_t current);
void SVPWM_Disable(foc_index_e index);
void SVPWM_Enable(foc_index_e index);
void FOC_Current_Plot(foc_index_e index);

void FOC_Current_Print_A_Cycle(foc_index_e index);
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

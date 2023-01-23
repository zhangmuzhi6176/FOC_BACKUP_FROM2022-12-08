#ifndef _ZMZ_FOC_5225_CONF_H
#define _ZMZ_FOC_5225_CONF_H

#include "zmz_foc.h"

static double _FOC_Torq_Bind_Simple_Liner_Generator_Head(double trend_bind_ratio, double val)
{
    return val * (1 - trend_bind_ratio);
}

static double _FOC_Torq_Bind_Simple_Liner_Generator_Tail(double trend_bind_ratio, double val)
{
    return val * trend_bind_ratio;
}

foc_dev_t foc_dev_conf_g[] = {
    [FOC_I] = {
        .name = "5225",
        .bldc_tim_idx = {0, 1, 2},
        .svpwm_enable = SVPWM_ENABLE,
        .cali_pole_num = 11,
        .encoder_idx = ENC_NO_1,

        .cali_max_cur = false,

        .current_adc_chnl[PHASE_A] = 3,
        .current_adc_chnl[PHASE_B] = 1,
        .current_adc_chnl[PHASE_C] = 2,

        .current_adc_offset[PHASE_A] = (-311),  /* Default value in case no _FOC_Current_Cali */
        .current_adc_offset[PHASE_B] = (-11),   /* Default value in case no _FOC_Current_Cali */
        .current_adc_offset[PHASE_C] = (-9),    /* Default value in case no _FOC_Current_Cali */

        .current_adc_idx = ADC_I,
        .current_max = {1350, 1500, 1500},      /* Default value in case no _FOC_Current_Cali */
        .current_min = {-1350, -1500, -1500},

        .Q_offset = 0.15,
        .Torq_I_val_l = 0.05,
        .Torq_I_val_h = 0.2,
        .foc_pid[FOC_PID_Q] = {
            .P = 1,
            .I = 0,
            .D = 0,
            .error_accum_max = 10,
        },
        .foc_pid[FOC_PID_D] = {
            .P = 1,
            .I = 0,
            .D = 0,
            .error_accum_max = 10,
        },
        .filter_bind_param[FOC_FILTER_BIND_TORQ] = {
            .start_bind_ratio = 0.05,
            .end_bind_ratio = 0.5,
            .trend_val_max = 1,
            .head_val_generator = _FOC_Torq_Bind_Simple_Liner_Generator_Head,
            .tail_val_generator = _FOC_Torq_Bind_Simple_Liner_Generator_Tail,
        },

        .foc_pid[FOC_PID_SPEED] = {
            .P = 0.0003,
            .I = 0.0001,
            .D = 0,
            .error_accum_max = 2500,
        },

        .foc_pid[FOC_PID_POSITION] = {
            .P = 1,
            .I = 0.015,
            .D = 0,
            .error_accum_max = 10,
        },
        .position_oscilation_detector = {
            .sign_change_cnt_thresh = 20,
            .sign_unchange_cnt_thresh = 255,
        },
    }
};

#endif

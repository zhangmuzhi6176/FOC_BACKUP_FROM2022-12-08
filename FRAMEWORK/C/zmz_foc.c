#include "zmz_foc.h"
#include "zmz_filter.h"
#include "zmz_led.h"
#include "zmz_pid.h"
#include "zmz_mt6701.h"

#include "zmz_delay.h"
#include "zmz_adc_drv_STM32F103.h"
#include "zmz_timer_hal.h"
#include "zmz_uart_hal.h"

#include <math.h>
#include <stdlib.h>

typedef struct svpwm_stage {
    double stage_short;
    double stage_middle;
    double stage_long;
} svpwm_stage_t;

typedef struct foc_dev {
    u8 bldc_tim_idx[PHASE_NUM];
    u8 encoder_idx;
    u8 cali_pole_num;

    svpwm_enable_e svpwm_enable;

    double cali_Magfield_2_Enc_angle_offset;

    u8 current_adc_chnl[PHASE_NUM];
    short current_adc_offset[PHASE_NUM];        /* Calibration required */
    u8 current_adc_idx;
    double current_last[PHASE_NUM];
    double current_max[PHASE_NUM];              /* Calibration required */

    PID_param_t foc_pid[FOC_PID_NUM];

    double mech_angle_last;
    double time_last;
    double speed_deg_per_seconds_max;

    double position_torq_factor;
} foc_dev_t;

foc_dev_t foc_dev_g[] = {
    [FOC_I] = {
        .bldc_tim_idx = {0, 1, 2},
        .svpwm_enable = SVPWM_ENABLE,
        .cali_pole_num = 11,
        .encoder_idx = ENC_NO_1,
        .current_adc_chnl[PHASE_A] = 3,
        .current_adc_chnl[PHASE_B] = 1,
        .current_adc_chnl[PHASE_C] = 2,
        .current_adc_offset[PHASE_A] = (286 - ADC_14_BIT_MID_VAL),
        .current_adc_offset[PHASE_B] = (-12 - ADC_14_BIT_MID_VAL),
        .current_adc_offset[PHASE_C] = (49 - ADC_14_BIT_MID_VAL),
        .current_adc_idx = ADC_I,
        .current_last = {0, 0, 0},
        .current_max = {1200, 1450, 1450},
        .foc_pid[FOC_PID_Q] = {
            .P = 0.9,
            .I = 0.1,
            .D = 0,
            .error_accum_max = 8,
        },
        .foc_pid[FOC_PID_D] = {
            .P = 0.9,
            .I = 0.1,
            .D = 0,
            .error_accum_max = 8,
        },
        .speed_deg_per_seconds_max = 2160,
        .foc_pid[FOC_PID_SPEED] = {
            .P = 1.15,
            .I = 0.12,
            .D = 0.3,
            .error_accum_max = 3,
        },
        .foc_pid[FOC_PID_POSITION] = {
            .P = 0.5,
            .I = 0,
            .D = 0.5,
            .error_accum_max = 0,
        },
        .position_torq_factor = 20,
    }
};

__attribute__((unused)) static double _FOC_Get_Mech_angle(foc_index_e index)
{
    double ret = MT_Get_ANGLE(foc_dev_g[index].encoder_idx) - foc_dev_g[index].cali_Magfield_2_Enc_angle_offset;
    if (ret < 0) {
        return ret + RAD_2_DEG(2 * PI);
    }
    return ret;
}

__attribute__((unused)) double _FOC_Get_Elec_angle(foc_index_e index)
{
    return _FOC_Get_Mech_angle(index) * foc_dev_g[index].cali_pole_num;
}

__attribute__((unused)) static double _FOC_Get_Current(foc_index_e index, bldc_phase_e phase)
{
    return (ADC_Get_Val_From_DMA_Drv(foc_dev_g[index].current_adc_idx, foc_dev_g[index].current_adc_chnl[phase]) + foc_dev_g[index].current_adc_offset[phase]);
}

__attribute__((unused)) static double _FOC_Value_Limit(double val, double limit_h, double limit_l)
{
    if (val >= limit_h) {
        return limit_h;
    } else if (val <= limit_l) {
        return limit_l;
    }
    return val;
}

__attribute__((unused)) static double _FOC_Angle_Diff_Abs(double angle_1, double angle_2, double full_angle)
{
    double angle_diff = fabs(angle_1 - angle_2);
    if (angle_diff >= (full_angle / 2)) {
        return (full_angle - angle_diff);
    } else {
        return angle_diff;
    }
}

__attribute__((unused)) static double _FOC_Angle_Diff_Sign(double angle_1, double angle_2, double full_angle)
{
    double angle_diff = angle_1 - angle_2;
    if (angle_diff >= (full_angle / 2)) {
        return (angle_diff - full_angle);
    } else if (angle_diff <= (full_angle / (-2))) {
        return (angle_diff + full_angle);
    }
    return angle_diff;
}

__attribute__((unused)) static double _FOC_Mech_Angle_In_Range(double mech_angle_deg)
{
    double mech_angle_deg_max = RAD_2_DEG(2 * PI);
    if (mech_angle_deg >= mech_angle_deg_max) {
        return fmod(mech_angle_deg, mech_angle_deg_max);
    } else if (mech_angle_deg < 0) {
        return mech_angle_deg + mech_angle_deg_max;
    }
    return mech_angle_deg;
}

__attribute__((unused)) static double _FOC_Elec_Angle_In_Range(foc_index_e index, double elec_angle_deg)
{
    double elec_angle_deg_max = RAD_2_DEG(2 * PI) * foc_dev_g[index].cali_pole_num;
    if (elec_angle_deg >= elec_angle_deg_max) {
        return fmod(elec_angle_deg, elec_angle_deg_max);
    } else if (elec_angle_deg < 0) {
        return elec_angle_deg + elec_angle_deg_max;
    }
    return elec_angle_deg;
}

__attribute__((unused)) static void _clark_transform(foc_current_t *current)
{
    current->I_alpha = (current->I_a - sin(PI / 6) * current->I_b - sin(PI / 6) * current->I_c) * 2 / 3;
    current->I_beta = (sin(PI / 3) * (current->I_b - current->I_c)) * 2 / 3;
}

__attribute__((unused)) static void _park_transform(double elec_angle_deg, foc_current_t *current)
{
    current->I_d = current->I_alpha * cos(DEG_2_RAD(elec_angle_deg)) + current->I_beta * sin(DEG_2_RAD(elec_angle_deg));
    current->I_q = -1 * current->I_alpha * sin(DEG_2_RAD(elec_angle_deg)) + current->I_beta * cos(DEG_2_RAD(elec_angle_deg));
}

__attribute__((unused)) static void _reverse_park_transform(double elec_angle_deg, foc_current_t *current)
{
    current->I_alpha = current->I_d * cos(DEG_2_RAD(elec_angle_deg)) - current->I_q * sin(DEG_2_RAD(elec_angle_deg));
    current->I_beta = current->I_d * sin(DEG_2_RAD(elec_angle_deg)) + current->I_q * cos(DEG_2_RAD(elec_angle_deg));
}

__attribute__((unused)) static void _FOC_Set_Offset_angle(foc_index_e index)
{
    /* Has to be initialized out of range of MT_Get_ANGLE(), or the calibration may be skipped. */
    double angle_cali = -1;

    SVPWM_Generate_Mech_Ang(index, 0, BLDC_MAX_TORQUE);
    delay_ms(500);
    for (u8 i = 0; angle_cali != MT_Get_ANGLE(foc_dev_g[index].encoder_idx); i++) {
        delay_ms(200);
        angle_cali = MT_Get_ANGLE(foc_dev_g[index].encoder_idx);
        ZSS_FOC_LOGI("cali round [%d], angle_cali: [%.3f].\r\n", i, angle_cali);
    }
    SVPWM_Generate_Mech_Ang(index, 0, BLDC_ZERO_TORQUE);
    foc_dev_g[index].cali_Magfield_2_Enc_angle_offset = angle_cali;
}

__attribute__((unused)) static void _SVPWM_Calc_Stage(double t_first, double t_second, svpwm_stage_t *stage)
{
    double stage_1, stage_2, stage_3, stage_4, stage_5, stage_6, stage_7, sum = 0;

    stage_2 = t_first;
    stage_3 = t_second;
    stage_6 = stage_2;
    stage_5 = stage_3;
    stage_4 = (1 - stage_2 * 2 - stage_3 * 2) / 2;
    stage_1 = stage_4 / 2;
    stage_7 = stage_1;
    sum = stage_1 + stage_2 + stage_3 + stage_4 + stage_5 + stage_6 + stage_7;
    stage->stage_long = (stage_2 + stage_3 + stage_4 + stage_5 + stage_6) / sum;
    stage->stage_middle = (stage_3 + stage_4 + stage_5) / sum;
    stage->stage_short = (stage_4) / sum;
}

void SVPWM_Generate_Elec_Ang(foc_index_e index, double elec_angle_deg, double svpwm_duty)
{
    if (SVPWM_ENABLE == foc_dev_g[index].svpwm_enable) {
        double factor_A, factor_B, factor_C, t_first, t_second = 0;
        svpwm_phase_e phase = SVPWM_PHASE_INITIAL;
        svpwm_stage_t stage = {0};

        phase = (svpwm_phase_e)((u8)(elec_angle_deg / (RAD_2_DEG(2 * PI) / SVPWM_PHASE_NUM)) % SVPWM_PHASE_NUM);
        t_first = sin((phase + 1) * PI / 3 - DEG_2_RAD(elec_angle_deg)) / 2;
        t_second = sin(DEG_2_RAD(elec_angle_deg) - phase * PI / 3) / 2;

        switch (phase) {
        case SVPWM_PHASE_I:
            _SVPWM_Calc_Stage(t_first, t_second, &stage);
            factor_A = stage.stage_long;
            factor_B = stage.stage_middle;
            factor_C = stage.stage_short;
            break;
        case SVPWM_PHASE_II:
            _SVPWM_Calc_Stage(t_second, t_first, &stage);
            factor_B = stage.stage_long;
            factor_A = stage.stage_middle;
            factor_C = stage.stage_short;
            break;
        case SVPWM_PHASE_III:
            _SVPWM_Calc_Stage(t_first, t_second, &stage);
            factor_B = stage.stage_long;
            factor_C = stage.stage_middle;
            factor_A = stage.stage_short;
            break;
        case SVPWM_PHASE_IV:
            _SVPWM_Calc_Stage(t_second, t_first, &stage);
            factor_C = stage.stage_long;
            factor_B = stage.stage_middle;
            factor_A = stage.stage_short;
            break;
        case SVPWM_PHASE_V:
            _SVPWM_Calc_Stage(t_first, t_second, &stage);
            factor_C = stage.stage_long;
            factor_A = stage.stage_middle;
            factor_B = stage.stage_short;
            break;
        case SVPWM_PHASE_VI:
            _SVPWM_Calc_Stage(t_second, t_first, &stage);
            factor_A = stage.stage_long;
            factor_C = stage.stage_middle;
            factor_B = stage.stage_short;
            break;
        default:
            ZSS_FOC_LOGE("SVPWM_Generate_Mech_Ang in unsupported phase [%d].\r\n", phase);
            Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
            return;
        }

        if (svpwm_duty > BLDC_MAX_TORQUE) {
            svpwm_duty = BLDC_MAX_TORQUE;
        } else if (svpwm_duty < BLDC_ZERO_TORQUE) {
            svpwm_duty = BLDC_ZERO_TORQUE;
        }

        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_A], factor_A * svpwm_duty);
        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_B], factor_B * svpwm_duty);
        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_C], factor_C * svpwm_duty);
    } else {
        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
    }
}

void SVPWM_Generate_Mech_Ang(foc_index_e index, double mech_angle_deg, double svpwm_duty)
{
    double elec_angle_deg = elec_angle_deg = mech_angle_deg * foc_dev_g[index].cali_pole_num;
    SVPWM_Generate_Elec_Ang(index, elec_angle_deg, svpwm_duty);
}

void SVPWM_Disable(foc_index_e index)
{
    foc_dev_g[index].svpwm_enable = SVPWM_DISABLE;
    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
}

void SVPWM_Enable(foc_index_e index)
{
    foc_dev_g[index].svpwm_enable = SVPWM_ENABLE;
}

void FOC_Print_Current(foc_index_e index)
{
    double current[PHASE_NUM] = {0};
    double current_max[PHASE_NUM] = {0};
    double current_min[PHASE_NUM] = {0};
    u8 PRINT_WIDTH = 8;

    for (double mech_angle_deg = 0; mech_angle_deg < RAD_2_DEG(2 * PI); mech_angle_deg += 0.1) {
        SVPWM_Generate_Mech_Ang(index, mech_angle_deg, BLDC_MAX_TORQUE);
        current[PHASE_A] = _FOC_Get_Current(index, PHASE_A);
        current[PHASE_B] = _FOC_Get_Current(index, PHASE_B);
        current[PHASE_C] = _FOC_Get_Current(index, PHASE_C);

        if (mech_angle_deg > RAD_2_DEG(PI / 6)) {
            if (current[PHASE_A] > current_max[PHASE_A]) {
                current_max[PHASE_A] = current[PHASE_A];
            }
            if (current[PHASE_B] > current_max[PHASE_B]) {
                current_max[PHASE_B] = current[PHASE_B];
            }
            if (current[PHASE_C] > current_max[PHASE_C]) {
                current_max[PHASE_C] = current[PHASE_C];
            }

            if (current[PHASE_A] < current_min[PHASE_A]) {
                current_min[PHASE_A] = current[PHASE_A];
            }
            if (current[PHASE_B] < current_min[PHASE_B]) {
                current_min[PHASE_B] = current[PHASE_B];
            }
            if (current[PHASE_C] < current_min[PHASE_C]) {
                current_min[PHASE_C] = current[PHASE_C];
            }
        }

        ZSS_FOC_LOGD("FOC_PRT_CUR", "A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                     PRINT_WIDTH, current[PHASE_A], PRINT_WIDTH, current[PHASE_B], PRINT_WIDTH, current[PHASE_C]);
        ZSS_FOC_LOGPLOT("%-*.3f, %-*.3f, %-*.3f\r\n",
               PRINT_WIDTH, current[PHASE_A] / 30, PRINT_WIDTH, current[PHASE_B] / 30, PRINT_WIDTH, current[PHASE_C] / 30);
    }

    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index].bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);

    ZSS_FOC_LOGI("MAX: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 PRINT_WIDTH, current_max[PHASE_A], PRINT_WIDTH, current_max[PHASE_B], PRINT_WIDTH, current_max[PHASE_C]);
    ZSS_FOC_LOGI("MIN: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 PRINT_WIDTH, current_min[PHASE_A], PRINT_WIDTH, current_min[PHASE_B], PRINT_WIDTH, current_min[PHASE_C]);
    ZSS_FOC_LOGI("MID: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 PRINT_WIDTH, (current_max[PHASE_A] + current_min[PHASE_A]) / 2,
                 PRINT_WIDTH, (current_max[PHASE_B] + current_min[PHASE_B]) / 2,
                 PRINT_WIDTH, (current_max[PHASE_C] + current_min[PHASE_C]) / 2);
    ZSS_FOC_LOGI("RANGE: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 PRINT_WIDTH, current_max[PHASE_A] - current_min[PHASE_A],
                 PRINT_WIDTH, current_max[PHASE_B] - current_min[PHASE_B],
                 PRINT_WIDTH, current_max[PHASE_C] - current_min[PHASE_C]);
}

void FOC_Init(void)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(foc_dev_g); i++) {
        _FOC_Set_Offset_angle((foc_index_e)i);
        for (u8 j = 0; j< ZSS_ARRAY_SIZE(foc_dev_g[i].foc_pid); j++) {
            PID_Init(&(foc_dev_g[i].foc_pid[j]));
        }
        ZSS_FOC_LOGI("FOC [%d] is initialized, pole num: [%d], Mag_2_Enc offset: [%.3f], MAX_cur A[%.3f], B[%.3f], C[%.3f].\r\n",
                     i, foc_dev_g[i].cali_pole_num, foc_dev_g[i].cali_Magfield_2_Enc_angle_offset,
                     foc_dev_g[i].current_max[PHASE_A], foc_dev_g[i].current_max[PHASE_B], foc_dev_g[i].current_max[PHASE_C]);
    }
}

static filter_1st_lp_t filter_1 = {
    .alpha = 0.1,
};

static filter_1st_lp_t filter_2 = {
    .alpha = 0.1,
};

void FOC_Keep_Torque(foc_index_e index, double Q_ref, double intensity)
{
    double elec_angle_deg_cur, elec_angle_deg_next, svpwm_duty = 0;
    foc_current_t current = {0};

    current.I_a = _FOC_Get_Current(index, PHASE_A) / foc_dev_g[index].current_max[PHASE_A];
    current.I_b = _FOC_Get_Current(index, PHASE_B) / foc_dev_g[index].current_max[PHASE_B];
    current.I_c = _FOC_Get_Current(index, PHASE_C) / foc_dev_g[index].current_max[PHASE_C];

    _clark_transform(&current);

    /* Reading fresh elec_angle here is necessary */
    elec_angle_deg_cur = _FOC_Get_Elec_angle(index);
    _park_transform(elec_angle_deg_cur, &current);

    /* current.I_d = Filter_1st_LP(&filter_1, current.I_d);
    current.I_q = Filter_1st_LP(&filter_2, current.I_q); */

    /* For debug only */
    ZSS_FOC_LOGPLOT("%-*.3f, %-*.3f, %-*.3f\r\n", FOC_PLOT_WIDTH, current.I_d * 40, FOC_PLOT_WIDTH, current.I_q * 40, FOC_PLOT_WIDTH, Q_ref * 40);

    /* current.I_d = PID_calc_I(&(foc_dev_g[index].foc_pid[FOC_PID_D]), 0, current.I_d);
    current.I_q = PID_calc_I(&(foc_dev_g[index].foc_pid[FOC_PID_Q]), Q_ref, current.I_q); */


    current.I_d = 0;
    current.I_q = Q_ref;


    /* Reading fresh elec_angle here is recommended but not necessary */
    elec_angle_deg_next = _FOC_Elec_Angle_In_Range(index, (_FOC_Get_Elec_angle(index) + RAD_2_DEG(atan2(current.I_q, current.I_d))));
    svpwm_duty = sqrt(current.I_q * current.I_q + current.I_d * current.I_d) * BLDC_MAX_TORQUE;

    /* svpwm_duty = Filter_1st_LP(&filter, svpwm_duty); */
    /* printf("%.3f, %.3f\r\n", Q_ref * 30, Filter_1st_LP(&filter, Q_ref) * 30); */

    SVPWM_Generate_Elec_Ang(index, elec_angle_deg_next, svpwm_duty * intensity);
        
    /* For debug only */
    /* ZSS_FOC_LOGPLOT("%-*.3f, %-*.3f\r\n", FOC_PLOT_WIDTH, RAD_2_DEG(atan2(current.I_q, current.I_d)), FOC_PLOT_WIDTH, svpwm_duty * intensity); */

    RGB_Led_Set_Color(RGB_LED_I, RGB_LED_LAKE_BLUE, svpwm_duty / 100);
}

void FOC_Keep_Speed(foc_index_e index, double speed_ratio_ref, double intensity)
{
    double time_cur, speed_cur, speed_ratio_cur, speed_ratio_next, mech_angle_deg_cur = 0;

    time_cur = Timer_Get_System_Time_Second_Drv();
    mech_angle_deg_cur = _FOC_Get_Mech_angle(index);
    speed_cur = _FOC_Angle_Diff_Sign(mech_angle_deg_cur, foc_dev_g[index].mech_angle_last, RAD_2_DEG(2 * PI)) / (time_cur - foc_dev_g[index].time_last);
    speed_ratio_cur = speed_cur / foc_dev_g[index].speed_deg_per_seconds_max;

    speed_ratio_next = PID_calc_I(&(foc_dev_g[index].foc_pid[FOC_PID_SPEED]), speed_ratio_ref, speed_ratio_cur);

    /* speed_ratio_next = _FOC_Value_Limit(speed_ratio_next, 1, -1); */

    FOC_Keep_Torque(index, speed_ratio_next, fabs(speed_ratio_next) * intensity);

    foc_dev_g[index].mech_angle_last = mech_angle_deg_cur;
    foc_dev_g[index].time_last = time_cur;

    /* For debug only */
    /* ZSS_FOC_LOGPLOT("%-*.3f, %-*.3f, %-*.3f\r\n", FOC_PLOT_WIDTH, speed_ratio_cur * 30, FOC_PLOT_WIDTH, speed_ratio_ref * 30, FOC_PLOT_WIDTH, speed_ratio_next * 30); */
}

void FOC_Keep_Position(foc_index_e index, double mech_angle_deg_ref, double intensity)
{
    double mech_angle_deg_cur, mech_angle_deg_delta, mech_angle_deg_next, torq_ratio_next = 0;

    mech_angle_deg_cur = _FOC_Get_Mech_angle(index);
    mech_angle_deg_delta = _FOC_Angle_Diff_Sign(mech_angle_deg_cur, mech_angle_deg_ref, RAD_2_DEG(2 * PI));
    mech_angle_deg_next = PID_calc_II(&(foc_dev_g[index].foc_pid[FOC_PID_POSITION]), 0, mech_angle_deg_delta);
    torq_ratio_next = mech_angle_deg_next / foc_dev_g[index].position_torq_factor;

    FOC_Keep_Torque(index, torq_ratio_next, intensity);

    /* For debug only */
    ZSS_FOC_LOGPLOT("%-*.3f, %-*.3f, %-*.3f\r\n", FOC_PLOT_WIDTH, mech_angle_deg_ref, FOC_PLOT_WIDTH, mech_angle_deg_cur, FOC_PLOT_WIDTH, mech_angle_deg_next);
}

void FOC_Go_Mech(foc_index_e index, double delta_mech_angle_deg, double speed_level, double intensity)
{}



/* 

位置环调参
    position_torq_factor也许可以去掉
    基本调参思路是：适当P，大D，零I

 */

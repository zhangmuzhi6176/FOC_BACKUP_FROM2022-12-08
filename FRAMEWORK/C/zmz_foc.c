#include "zmz_foc.h"

#include "zmz_delay.h"
#include "zmz_adc_drv_STM32F103.h"
#include "zmz_timer_hal.h"
#include "zmz_uart_hal.h"

#include <math.h>
#include <stdlib.h>

/* Specify different BLDC parameter configuration by including different header. */
#include "FOC_5225_conf.h"
foc_dev_t **foc_dev_g;

__attribute__((unused)) static double _FOC_Get_Mech_angle(foc_index_e index)
{
    double ret = MT_Get_ANGLE(foc_dev_g[index]->encoder_idx) - foc_dev_g[index]->cali_Magfield_2_Enc_angle_offset;
    if (ret < 0) {
        ret += RAD_2_DEG(2 * PI);
    }

    return ret;
}

__attribute__((unused)) double _FOC_Get_Elec_angle(foc_index_e index)
{
    return _FOC_Get_Mech_angle(index) * foc_dev_g[index]->cali_pole_num;
}

__attribute__((unused)) static double _FOC_Get_Current(foc_index_e index, bldc_phase_e phase)
{
    return (ADC_Get_Val_From_DMA_Drv(foc_dev_g[index]->current_adc_idx, foc_dev_g[index]->current_adc_chnl[phase]) - foc_dev_g[index]->current_adc_offset[phase] - ADC_14_BIT_MID_VAL);
}

__attribute__((unused)) static double _FOC_Get_Current_Derive(foc_index_e index, bldc_phase_e phase)
{
    double cur[PHASE_NUM] = {0};
    double ret = 0;
    cur[PHASE_A] = _FOC_Get_Current(index, PHASE_A);
    cur[PHASE_B] = _FOC_Get_Current(index, PHASE_B);
    cur[PHASE_C] = _FOC_Get_Current(index, PHASE_C);
    for (u16 i = 0; i < PHASE_NUM; i++) {
        if (i != phase) {
            ret += cur[i];
        }
    }
    return (-1 * ret);
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
    double elec_angle_deg_max = RAD_2_DEG(2 * PI) * foc_dev_g[index]->cali_pole_num;
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
    for (u8 i = 0; angle_cali != MT_Get_ANGLE(foc_dev_g[index]->encoder_idx); i++) {
        delay_ms(200);
        angle_cali = MT_Get_ANGLE(foc_dev_g[index]->encoder_idx);
        ZSS_FOC_LOGI("cali round [%d], angle_cali: [%.3f].\r\n", i, angle_cali);
    }
    SVPWM_Generate_Mech_Ang(index, 0, BLDC_ZERO_TORQUE);
    foc_dev_g[index]->cali_Magfield_2_Enc_angle_offset = angle_cali;
}

__attribute__((unused)) static double _FOC_Map_Value(double input, double start_val_before_map, double end_val_before_map, double start_val_after_map, double end_val_after_map)
{
    double range = fabs(start_val_before_map - end_val_before_map);
    double diff = fabs(fabs(start_val_before_map - end_val_before_map) - fabs(start_val_after_map - end_val_after_map));
    if (input <= end_val_before_map && input >= start_val_before_map) {
        return (input * ((range - diff) / range) + diff);
    } else if (input >= (-1 * end_val_before_map) && input < start_val_before_map) {
        return (input * ((range - diff) / range) - diff);
    } else {
        return input;
    }
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
    if (SVPWM_ENABLE == foc_dev_g[index]->svpwm_enable) {
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
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
            ZSS_ASSERT_WITH_LOG("SVPWM_Generate_Mech_Ang in unsupported phase [%d].\r\n", phase);
        }

        if (svpwm_duty > BLDC_MAX_TORQUE) {
            svpwm_duty = BLDC_MAX_TORQUE;
        } else if (svpwm_duty < BLDC_ZERO_TORQUE) {
            svpwm_duty = BLDC_ZERO_TORQUE;
        }

        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], factor_A * svpwm_duty);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], factor_B * svpwm_duty);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], factor_C * svpwm_duty);
    } else {
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
    }
}

void SVPWM_Generate_Mech_Ang(foc_index_e index, double mech_angle_deg, double svpwm_duty)
{
    double elec_angle_deg = elec_angle_deg = mech_angle_deg * foc_dev_g[index]->cali_pole_num;
    SVPWM_Generate_Elec_Ang(index, elec_angle_deg, svpwm_duty);
}

void SVPWM_Disable(foc_index_e index)
{
    foc_dev_g[index]->svpwm_enable = SVPWM_DISABLE;
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
}

void SVPWM_Enable(foc_index_e index)
{
    foc_dev_g[index]->svpwm_enable = SVPWM_ENABLE;
}

void FOC_Current_Print_A_Cycle(foc_index_e index)
{
    double current[PHASE_NUM] = {0};
    double current_max[PHASE_NUM] = {0};
    double current_min[PHASE_NUM] = {0};

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

        ZSS_FOC_LOGPLOT_TRIPPLE(current[PHASE_A], current[PHASE_B], current[PHASE_C], 0.03);
    }

    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);

    ZSS_FOC_LOGI("MAX: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 FOC_PLOT_WIDTH, current_max[PHASE_A], FOC_PLOT_WIDTH, current_max[PHASE_B], FOC_PLOT_WIDTH, current_max[PHASE_C]);
    ZSS_FOC_LOGI("MIN: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 FOC_PLOT_WIDTH, current_min[PHASE_A], FOC_PLOT_WIDTH, current_min[PHASE_B], FOC_PLOT_WIDTH, current_min[PHASE_C]);
    ZSS_FOC_LOGI("MID: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 FOC_PLOT_WIDTH, (current_max[PHASE_A] + current_min[PHASE_A]) / 2,
                 FOC_PLOT_WIDTH, (current_max[PHASE_B] + current_min[PHASE_B]) / 2,
                 FOC_PLOT_WIDTH, (current_max[PHASE_C] + current_min[PHASE_C]) / 2);
    ZSS_FOC_LOGI("RANGE: A[%-*.3f], B[%-*.3f], C[%-*.3f]\r\n",
                 FOC_PLOT_WIDTH, current_max[PHASE_A] - current_min[PHASE_A],
                 FOC_PLOT_WIDTH, current_max[PHASE_B] - current_min[PHASE_B],
                 FOC_PLOT_WIDTH, current_max[PHASE_C] - current_min[PHASE_C]);
}

__attribute__((unused)) static void _FOC_Max_Current_Cali(foc_index_e index)
{
    double cur = 0;
    u16 current_max_cali_times = 150;

    /* Calibrate FOC Phase Max current */
    for (u16 i = 0; i < PHASE_NUM; i++) {
        cur = 0;

        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);

        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[i], BLDC_MAX_TORQUE);
        delay_ms(10);
        for (u16 j = 0; j < current_max_cali_times; j++) {
            cur += _FOC_Get_Current(index, (bldc_phase_e)i);
        }
        foc_dev_g[index]->current_max[i] = cur / current_max_cali_times;
        delay_ms(300);
    }

    /* Calibrate FOC Phase Min current */
    for (u16 i = 0; i < PHASE_NUM; i++) {
        cur = 0;

        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_MAX_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_MAX_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_MAX_TORQUE);

        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[i], BLDC_ZERO_TORQUE);
        delay_ms(10);
        for (u16 j = 0; j < current_max_cali_times; j++) {
            cur += _FOC_Get_Current(index, (bldc_phase_e)i);
        }
        foc_dev_g[index]->current_min[i] = cur / current_max_cali_times;
        delay_ms(300);
    }

    delay_ms(500);
}

static void _FOC_Current_Cali(foc_index_e index)
{
    double current[PHASE_NUM] = {0};
    u16 current_offset_cali_times = 2000;

    SVPWM_Generate_Elec_Ang(index, 0, BLDC_ZERO_TORQUE);
    delay_ms(1500);

    /* Calibrate FOC Phase Offset current */
    for (u16 i = 0; i < PHASE_NUM; i++) {
        for (u16 j = 0; j < current_offset_cali_times; j++) {
            current[i] += ADC_Get_Val_From_DMA_Drv(foc_dev_g[index]->current_adc_idx, foc_dev_g[index]->current_adc_chnl[i]) - ADC_14_BIT_MID_VAL;
        }
        foc_dev_g[index]->current_adc_offset[i] = current[i] / current_offset_cali_times;
    }
    ZSS_FOC_LOGI("FOC [%d] Cur_offset A[%.3f], B[%.3f], C[%.3f].\r\n", index,
                 foc_dev_g[index]->current_adc_offset[PHASE_A], foc_dev_g[index]->current_adc_offset[PHASE_B], foc_dev_g[index]->current_adc_offset[PHASE_C]);

    if (foc_dev_g[index]->cali_max_cur) {
        _FOC_Max_Current_Cali(index);
    }

    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
    Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);

    ZSS_FOC_LOGI("FOC [%d] MAX_cur A[%.3f], B[%.3f], C[%.3f], MIN_cur A[%.3f], B[%.3f], C[%.3f].\r\n", index,
                 foc_dev_g[index]->current_max[PHASE_A], foc_dev_g[index]->current_max[PHASE_B], foc_dev_g[index]->current_max[PHASE_C],
                 foc_dev_g[index]->current_min[PHASE_A], foc_dev_g[index]->current_min[PHASE_B], foc_dev_g[index]->current_min[PHASE_C]);

    if ((fabs(fabs(foc_dev_g[index]->current_max[PHASE_A]) - fabs(foc_dev_g[index]->current_min[PHASE_A])) >= BLDC_CURRENT_CALI_DIFF_THRESH)
        || (fabs(fabs(foc_dev_g[index]->current_max[PHASE_B]) - fabs(foc_dev_g[index]->current_min[PHASE_B])) >= BLDC_CURRENT_CALI_DIFF_THRESH)
        || (fabs(fabs(foc_dev_g[index]->current_max[PHASE_C]) - fabs(foc_dev_g[index]->current_min[PHASE_C])) >= BLDC_CURRENT_CALI_DIFF_THRESH)) {
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
        Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
        ZSS_ASSERT_WITH_LOG("FOC [%d] current_max and current_min abnormal!!! A [%.3f] B [%.3f] C [%.3f]\r\n",
                            index,
                            fabs(foc_dev_g[index]->current_max[PHASE_A] - fabs(foc_dev_g[index]->current_min[PHASE_A])),
                            fabs(foc_dev_g[index]->current_max[PHASE_B] - fabs(foc_dev_g[index]->current_min[PHASE_B])),
                            fabs(foc_dev_g[index]->current_max[PHASE_C] - fabs(foc_dev_g[index]->current_min[PHASE_C])));
    }
}

void FOC_Current_Plot(foc_index_e index)
{
    foc_current_t current = {0};

    SVPWM_Generate_Elec_Ang(index, 0, BLDC_ZERO_TORQUE);

    current.I_a = _FOC_Get_Current(index, PHASE_A);
    current.I_b = _FOC_Get_Current(index, PHASE_B);
    current.I_c = _FOC_Get_Current(index, PHASE_C);

    ZSS_FOC_LOGPLOT_TRIPPLE(current.I_a, current.I_b, current.I_c, 1);
}

void FOC_Init(void)
{
    u8 foc_arry_size = 0;
    foc_arry_size = ZSS_ARRAY_SIZE(foc_dev_conf_g);
    foc_dev_g = (foc_dev_t **)malloc(foc_arry_size * sizeof(foc_dev_t *));
    for (u32 i = 0; i < foc_arry_size; i++) {
        foc_dev_g[i] = &(foc_dev_conf_g[i]);
    }

    for (u32 i = 0; i < foc_arry_size; i++) {
        _FOC_Current_Cali((foc_index_e)i);
        _FOC_Set_Offset_angle((foc_index_e)i);
        for (u8 j = 0; j < ZSS_ARRAY_SIZE(foc_dev_g[i]->foc_pid); j++) {
            PID_Init(&(foc_dev_g[i]->foc_pid[j]));
        }
        ZSS_FOC_LOGI("FOC [%s][%d] is initialized, pole num: [%d], Mag_2_Enc offset: [%.3f], MAX_cur A[%.3f], B[%.3f], C[%.3f], Cur_offset A[%.3f], B[%.3f], C[%.3f].\r\n",
                     foc_dev_g[i]->name, i, foc_dev_g[i]->cali_pole_num, foc_dev_g[i]->cali_Magfield_2_Enc_angle_offset,
                     foc_dev_g[i]->current_max[PHASE_A], foc_dev_g[i]->current_max[PHASE_B], foc_dev_g[i]->current_max[PHASE_C],
                     foc_dev_g[i]->current_adc_offset[PHASE_A], foc_dev_g[i]->current_adc_offset[PHASE_B], foc_dev_g[i]->current_adc_offset[PHASE_C]);
    }
}

/* 
 * arg_1 don't care     arg_2 = 0           false
 * arg_1 = 0            arg_2 don't care    false
 */
__attribute__((unused)) static bool _FOC_Check_sign_Changed_Strong(double arg_1, double arg_2)
{
    return (((arg_1 * arg_2) >= 0) ? false : true);
}

/* 
 * arg_1 don't care     arg_2 = 0           true
 * arg_1 = 0            arg_2 don't care    true
 */
__attribute__((unused)) static bool _FOC_Check_sign_Changed_Weak(double arg_1, double arg_2)
{
    return (((arg_1 * arg_2) > 0) ? false : true);
}

__attribute__((unused)) static bool _FOC_Check_Qref_sign_Changed(foc_index_e index, double Q_ref)
{
    bool ret = false;
    
    ret = _FOC_Check_sign_Changed_Weak(Q_ref, foc_dev_g[index]->Q_ref_last);
    foc_dev_g[index]->Q_ref_last = Q_ref;

    return ret;
}

void FOC_Keep_Torque(foc_index_e index, double Q_ref)
{
    double elec_angle_deg_cur, elec_angle_deg_next, svpwm_duty, I_d_next, I_q_next, Q_offset = 0;
    foc_current_t current = {0};

    if (Q_ref > 0) {
        Q_offset = foc_dev_g[index]->Q_offset;
    } else if (Q_ref < 0) {
        Q_offset = (-1 * foc_dev_g[index]->Q_offset);
    }

    current.I_a = _FOC_Get_Current(index, PHASE_A) / foc_dev_g[index]->current_max[PHASE_A];
    current.I_b = _FOC_Get_Current(index, PHASE_B) / foc_dev_g[index]->current_max[PHASE_B];
    current.I_c = _FOC_Get_Current(index, PHASE_C) / foc_dev_g[index]->current_max[PHASE_C];

    _clark_transform(&current);

    /* Reading fresh elec_angle here is necessary */
    elec_angle_deg_cur = _FOC_Get_Elec_angle(index);
    _park_transform(elec_angle_deg_cur, &current);
    foc_dev_g[index]->mech_angle_current = elec_angle_deg_cur / foc_dev_g[index]->cali_pole_num;

    /* For debug only */
    /* ZSS_FOC_LOGPLOT_TRIPPLE(current.I_d, current.I_q, Q_ref, 100); */

    foc_dev_g[index]->foc_pid[FOC_PID_Q].I = Filter_Bind(foc_dev_g[index]->filter_bind_param[FOC_FILTER_BIND_TORQ], foc_dev_g[index]->Torq_I_val_l, foc_dev_g[index]->Torq_I_val_h, fabs(Q_ref));
    foc_dev_g[index]->foc_pid[FOC_PID_D].I = Filter_Bind(foc_dev_g[index]->filter_bind_param[FOC_FILTER_BIND_TORQ], foc_dev_g[index]->Torq_I_val_l, foc_dev_g[index]->Torq_I_val_h, fabs(Q_ref));

    I_d_next = current.I_d + PID_calc_Pos(&(foc_dev_g[index]->foc_pid[FOC_PID_D]), 0, current.I_d);
    I_q_next = Q_offset + current.I_q + PID_calc_Pos(&(foc_dev_g[index]->foc_pid[FOC_PID_Q]), Q_ref, current.I_q);

    /* Reading fresh elec_angle here is recommended but not necessary */
    elec_angle_deg_next = _FOC_Elec_Angle_In_Range(index, (elec_angle_deg_cur + RAD_2_DEG(atan2(I_q_next, I_d_next))));
    svpwm_duty = sqrt(I_q_next * I_q_next + I_d_next * I_d_next) * BLDC_MAX_TORQUE;
    SVPWM_Generate_Elec_Ang(index, elec_angle_deg_next, svpwm_duty);

    /* For debug only */
    /* ZSS_FOC_LOGPLOT_TRIPPLE(elec_angle_deg_next, svpwm_duty, Q_ref, 1); */

    if (_FOC_Check_Qref_sign_Changed(index, Q_ref)) {
        PID_Flush_Error_Accum(&(foc_dev_g[index]->foc_pid[FOC_PID_D]));
        PID_Flush_Error_Accum(&(foc_dev_g[index]->foc_pid[FOC_PID_Q]));
    }

    RGB_Led_Set_Color(RGB_LED_I, RGB_LED_LAKE_BLUE, svpwm_duty / 100);
}

void FOC_Keep_Speed(foc_index_e index, double speed_ref)
{
    double mech_angle_current, time_cur, speed_cur, torq_next = 0;
    bldc_direction_e dir = BLDC_STOP;

    if (speed_ref > 0) {
        dir = BLDC_CW;
    } else if (speed_ref < 0) {
        dir = BLDC_CCW;
    }

    time_cur = Timer_Get_System_Time_Second_Drv();
    mech_angle_current = _FOC_Get_Mech_angle(index);
    speed_cur = _FOC_Angle_Diff_Sign(mech_angle_current, foc_dev_g[index]->mech_angle_last, RAD_2_DEG(2 * PI)) / (time_cur - foc_dev_g[index]->time_last);
    torq_next = PID_calc_Pos(&(foc_dev_g[index]->foc_pid[FOC_PID_SPEED]), speed_ref, speed_cur);

    switch (dir) {
        case BLDC_STOP:
            FOC_Keep_Torque(index, BLDC_ZERO_TORQUE);
            break;
        case BLDC_CW:
            torq_next = _FOC_Value_Limit(torq_next, 1, 0.0001);
            break;
        case BLDC_CCW:
            torq_next = _FOC_Value_Limit(torq_next, -0.0001, -1);
            break;
        default:
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_A], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_B], BLDC_ZERO_TORQUE);
            Timer_Set_Duty_Hal(foc_dev_g[index]->bldc_tim_idx[PHASE_C], BLDC_ZERO_TORQUE);
            ZSS_ASSERT_WITH_LOG("Invalid param dir[%d].\r\n", dir);
    }
    FOC_Keep_Torque(index, torq_next);

    foc_dev_g[index]->time_last = time_cur;
    foc_dev_g[index]->mech_angle_last = mech_angle_current;

    /* For debug only */
    /* ZSS_FOC_LOGPLOT_TRIPPLE(speed_cur, speed_ref, torq_next, 0.1); */
}

__attribute__((unused)) static u32 _FOC_Supress_Oscilation(foc_index_e index, oscilation_detect_t *oscilation_detector, double subject)
{
    if (_FOC_Check_sign_Changed_Strong(subject, oscilation_detector->subject_last)) {
        /* subject sign changed */
        if (oscilation_detector->sign_unchanged_count < oscilation_detector->sign_change_cnt_thresh) {
            /* count oscilation */
            oscilation_detector->oscilation_count++;
            oscilation_detector->sign_unchanged_count = 0;
        }
        /* flush sign_unchanged_count */
        oscilation_detector->sign_unchanged_count = 0;
    } else {
        /* subject sign unchanged */
        oscilation_detector->sign_unchanged_count++;
    }
    
    if (oscilation_detector->sign_unchanged_count >= oscilation_detector->sign_unchange_cnt_thresh) {
        /* flush oscilation_count */
        oscilation_detector->oscilation_count = 0;
    }

    oscilation_detector->subject_last = subject;
    return oscilation_detector->oscilation_count;
}

void FOC_Keep_Position(foc_index_e index, double mech_angle_deg_ref, double intensity)
{
    double mech_angle_deg_delta_ratio, torq_ratio_next, mech_angle_deg_delta = 0;

    mech_angle_deg_delta = _FOC_Angle_Diff_Sign(foc_dev_g[index]->mech_angle_current, mech_angle_deg_ref, RAD_2_DEG(2 * PI));
    mech_angle_deg_delta_ratio = mech_angle_deg_delta / RAD_2_DEG(PI);
    torq_ratio_next = PID_calc_Pos(&(foc_dev_g[index]->foc_pid[FOC_PID_POSITION]), 0, mech_angle_deg_delta_ratio);

    FOC_Keep_Torque(index, torq_ratio_next * intensity);

    /* For debug only */
    /* ZSS_FOC_LOGPLOT_TRIPPLE(mech_angle_deg_ref, mech_angle_current, mech_angle_deg_delta, 0.1); */
}

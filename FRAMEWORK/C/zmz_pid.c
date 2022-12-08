#include "zmz_pid.h"
#include <stdlib.h>
#include <math.h>

PID_param_t *PID_Init_Param(double P, double I, double D)
{
    PID_param_t *pid = (PID_param_t *)malloc(sizeof(PID_param_t));
    memset(pid, 0, sizeof(PID_param_t));
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->error_accum = 0;
    pid->last_error = 0;
    ZSS_PID_LOGI("PID param is initialized as: P[%.3f], I[%.3f], D[%.3f].\r\n",
                 pid->P, pid->I, pid->D);
    return pid;
}

void PID_Init(PID_param_t *pid)
{
    pid->error_accum = 0;
    pid->last_error = 0;
    ZSS_PID_LOGI("PID param is initialized as: P[%.3f], I[%.3f], D[%.3f].\r\n",
                 pid->P, pid->I, pid->D);
}

double PID_calc(PID_param_t *param, double ref_val, double cur_val)
{
    double rlt, error = 0;
    error = ref_val - cur_val;

    rlt = cur_val + param->P * error + param->I * param->error_accum + param->D * (error - param->last_error);
    param->last_error = error;

    if (fabs(param->error_accum + error) < param->error_accum_max) {
        param->error_accum += error;
    }

    return rlt;
}

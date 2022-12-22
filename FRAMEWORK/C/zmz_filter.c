#include "zmz_filter.h"
#include <stdlib.h>
#include <math.h>

/* 
TODO:
    增加init函数，malloc window
    调查稳态误差
     */
double Filter_Moving_Average(filter_moving_average_t *filter, double val)
{ 
    double rlt = 0;

    filter->window[filter->index] = val;
    if (filter->index == filter->window_size - 1) {
        filter->sum -= filter->window[0];
    } else {
        filter->sum -= filter->window[filter->index + 1];
    }

    filter->sum += val;
    rlt = filter->sum / filter->window_size;

    filter->index++;
    if (filter->index >= filter->window_size) {
        filter->index = 0;
    }

    /* for (int i = 0; i < filter->window_size; i++) {
        printf("%.3f, \r\n", filter->window[i] * 30);
    }
    printf("\r\n"); */

    return rlt;
}

double Filter_1st_LP(filter_1st_lp_t *filter, double val)
{
    double ret = 0;

    ret = filter->val_last + filter->alpha * (val - filter->val_last);
    filter->val_last = ret;

    return ret;
}

static double _Filter_Bind_Simple_Liner_Generator_Head(double trend_bind_ratio, double val)
{
    return val * (1 - trend_bind_ratio);
}

static double _Filter_Bind_Simple_Liner_Generator_Tail(double trend_bind_ratio, double val)
{
    return val * trend_bind_ratio;
}

double Filter_Bind(filter_bind_t bind_param, double val_head, double val_tail, double trend_val_cur)
{
    double bind_range_ratio, trend_ratio, trend_bind_ratio = 0;

    bind_range_ratio = bind_param.end_bind_ratio - bind_param.start_bind_ratio;
    trend_ratio = trend_val_cur / bind_param.trend_val_max;
    trend_bind_ratio = (trend_ratio - bind_param.start_bind_ratio) / bind_range_ratio;              /* indicates the progress ratio in bind range */

    if (!bind_param.head_val_generator) {
        bind_param.head_val_generator = _Filter_Bind_Simple_Liner_Generator_Head;
    }

    if (!bind_param.tail_val_generator) {
        bind_param.tail_val_generator = _Filter_Bind_Simple_Liner_Generator_Tail;
    }

    if (trend_ratio < bind_param.start_bind_ratio) {
        return val_head;
    } else if (trend_ratio >= bind_param.start_bind_ratio && trend_ratio <= bind_param.end_bind_ratio) {
        return bind_param.head_val_generator(trend_bind_ratio, val_head) + bind_param.tail_val_generator(trend_bind_ratio, val_tail);
    } else {
        return val_tail;
    }
}

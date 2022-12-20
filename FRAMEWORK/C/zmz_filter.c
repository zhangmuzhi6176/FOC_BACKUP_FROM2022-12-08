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

/* 优化得到sum的逻辑 */

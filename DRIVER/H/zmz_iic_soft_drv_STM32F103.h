#ifndef _IIC_SOFT_DRV_STM32F103_H
#define _IIC_SOFT_DRV_STM32F103_H

#include "zmz_gpio_drv_STM32F103.h"
#include "zmz_system.h"

#define ZSS_IIC_SOFT_LOGD(KEY, format, ...) ZSS_LOGD("IIC_SOFT", KEY, format, ##__VA_ARGS__)
#define ZSS_IIC_SOFT_LOGI(format, ...) ZSS_LOGI("IIC_SOFT", format, ##__VA_ARGS__)
#define ZSS_IIC_SOFT_LOGW(format, ...) ZSS_LOGW("IIC_SOFT", format, ##__VA_ARGS__)
#define ZSS_IIC_SOFT_LOGE(format, ...) ZSS_LOGE("IIC_SOFT", format, ##__VA_ARGS__)
#define ZSS_IIC_SOFT_LOGF(format, ...) ZSS_LOGF("IIC_SOFT", format, ##__VA_ARGS__)

#define IIC_CASE_GPIO_INIT_LOG(CASE, PIN, FUMCTION)                                                     \
    {                                                                                                   \
        CASE_GPIO_INIT_LOG(CASE, PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, FUMCTION) \
    }

#define IIC_CASE_SET_GPIO_CR(CASE, PIN, VAL)                  \
    {                                                         \
    case CASE:                                                \
        if (8 > PIN)                                         \
        {                                                     \
            GPIO##CASE##->CRL &= ~(0xF << (PIN * 4));         \
            GPIO##CASE##->CRL |= (u32)VAL << (PIN * 4);       \
        }                                                     \
        else                                                  \
        {                                                     \
            GPIO##CASE##->CRH &= ~(0xF << ((PIN - 8) * 4));   \
            GPIO##CASE##->CRH |= (u32)VAL << ((PIN - 8) * 4); \
        }                                                     \
        break;                                                \
    }

void Set_Sda_Out(u8 index);
void Set_Sda_In(u8 index);

u8 Get_Sda_IOgrp(u8 index);
u8 Get_Sda_IOnum(u8 index);
u8 Get_Scl_IOgrp(u8 index);
u8 Get_Scl_IOnum(u8 index);

void Set_IIC_Dev_Addr_Drv(u8 index, u8 addr);
u8 Get_IIC_Dev_Addr_Drv(u8 index);

void IIC_Init_Drv(void);

#endif

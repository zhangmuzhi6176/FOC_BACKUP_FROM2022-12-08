#ifndef _ZMZ_CAN_DRV_STM32F103_H
#define _ZMZ_CAN_DRV_STM32F103_H

#include "zmz_system_hardware.h"
#include "stm32f1xx_hal.h"

#define ZSS_CAN_LOGD(KEY, format, ...) ZSS_LOGD("CAN", KEY, format, ##__VA_ARGS__)
#define ZSS_CAN_LOGI(format, ...) ZSS_LOGI("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGW(format, ...) ZSS_LOGW("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGE(format, ...) ZSS_LOGE("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGF(format, ...) ZSS_LOGF("CAN", format, ##__VA_ARGS__)

#define CAN_TX_CASE_GPIO_INIT(CASE, PIN)                                              \
    {                                                                                 \
        CASE_GPIO_INIT(CASE, PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH) \
    }

#define CAN_RX_CASE_GPIO_INIT(CASE, PIN)                                                 \
    {                                                                                    \
        CASE_GPIO_INIT(CASE, PIN, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH) \
    }

#define CAN_BUF_SIZE                        8
#define CAN_TEMPLATE_FILTER_NUM             4   

extern CAN_FilterTypeDef CAN_filter_conf_template;

typedef enum can_index {
    CAN_I = 0,
} can_index_e;

typedef enum can_frame_type {
    CAN_DATA = CAN_RTR_DATA,
    CAN_REMOTE = CAN_RTR_REMOTE,
} can_frame_type_e;

typedef enum can_ID_type {
    CAN_STID = CAN_ID_STD,
    CAN_EXID = CAN_ID_EXT,
} can_ID_type_e;

typedef enum can_filter_type {
    CAN_LIST_MODE = CAN_FILTERMODE_IDLIST,
    CAN_MASK_MODE = CAN_FILTERMODE_IDMASK,
} can_filter_type_e;

typedef enum can_filter_size {
    CAN_16_BIT = CAN_FILTERSCALE_16BIT,
    CAN_32_BIT = CAN_FILTERSCALE_32BIT,
} can_filter_size_e;

void CAN_Init_Drv(void);
void CAN_Config_Tx_Drv(can_index_e index, can_frame_type_e frame_type, can_ID_type_e ID_type);
HAL_StatusTypeDef CAN_Config_Rx_Drv(can_index_e index, CAN_FilterTypeDef CAN_filter_conf);
HAL_StatusTypeDef CAN_Send(can_index_e index, u8 *tx_data);

#endif

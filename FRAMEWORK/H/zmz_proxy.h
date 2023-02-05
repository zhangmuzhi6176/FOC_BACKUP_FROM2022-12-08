#ifndef _ZMZ_PROXY_H
#define _ZMZ_PROXY_H

#include "zmz_uart_hal.h"
#include "zmz_can_drv_STM32F103.h"
#include "zmz_cmd_define.h"
#include "zmz_system_hardware.h"

#define ZSS_PROXY_LOGD(KEY, format, ...) ZSS_LOGD("PROXY", KEY, format, ##__VA_ARGS__)
#define ZSS_PROXY_LOGI(format, ...) ZSS_LOGI("PROXY", format, ##__VA_ARGS__)
#define ZSS_PROXY_LOGW(format, ...) ZSS_LOGW("PROXY", format, ##__VA_ARGS__)
#define ZSS_PROXY_LOGE(format, ...) ZSS_LOGE("PROXY", format, ##__VA_ARGS__)
#define ZSS_PROXY_LOGF(format, ...) ZSS_LOGF("PROXY", format, ##__VA_ARGS__)

#define MESSAGE_BUF_SIZE            CAN_BUF_SIZE

typedef enum proxy_index {
    PROXY_CAN = 0,
} proxy_index_e;

typedef struct proxy_dev {
    can_index_e                     can_index;
    void                          (*Proxy_Response_Func)(u32, u8 *);
} proxy_dev_t;

void Proxy_Response_CAN(u32 CAN_ID, u8 *arg);
proxy_dev_t *Get_Proxy_By_CAN_idx(can_index_e can_idx);
zss_return_e Zss_Proxy_Send(proxy_index_e proxy_index, ZSS_CMD_ID_e cmd_id, void *arg);
zss_return_e Zss_Proxy_Send_Retry(proxy_index_e proxy_index, ZSS_CMD_ID_e cmd_id, void *arg, u8 retry_times);

#endif

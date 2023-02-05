#include "zmz_proxy.h"

#include "zmz_can_drv_STM32F103.h"

typedef struct proxy_message {
    zss_return_e(*message_func)(u8 *arg);
} proxy_message_t;

void Proxy_Response_CAN(u32 CAN_ID, u8 *arg);
static proxy_dev_t proxy_dev_g[] = {
    [PROXY_CAN] = {
        .can_index = CAN_I,
        .Proxy_Response_Func = Proxy_Response_CAN,
    }
};

proxy_dev_t *Get_Proxy_By_CAN_idx(can_index_e can_idx)
{
    for (u8 i = 0; i < ZSS_ARRAY_SIZE(proxy_dev_g); i++) {
        if (can_idx == proxy_dev_g[i].can_index) {
            return &(proxy_dev_g[i]);
        }
    }

    return NULL;
}


/* --------------------------------------------------------------- Proxy_Response_CAN --------------------------------------------------------------- */

static zss_return_e _Proxy_FOC_Position(u8 *arg)
{
    foc_ctrl_t foc_pos;
    memcpy(&foc_pos, arg, sizeof(foc_ctrl_t));

    ZSS_PROXY_LOGI("foc_pos.mech_angle: %f\r\n", foc_pos.mech_angle);

    return SUCCESS_ZMZ;
}

static proxy_message_t _can_message_tbl[MESSAGE_BUF_SIZE] = {
    [ZSS_CMD_FOC_CTRL] = {.message_func = _Proxy_FOC_Position,},
};

void Proxy_Response_CAN(u32 CAN_ID, u8 *arg)
{
    _can_message_tbl[CAN_ID].message_func(arg);
}

/* --------------------------------------------------------------- Proxy_Response_CAN --------------------------------------------------------------- */


zss_return_e Zss_Proxy_Send(proxy_index_e proxy_index, ZSS_CMD_ID_e cmd_id, void *arg)
{
    int rlt = HAL_OK;
    u8 arg_len = 0;

    arg_len = ZSS_Get_Cmd_Param_Size(cmd_id);
    if (CAN_BUF_SIZE < arg_len) {
        ZSS_ASSERT_WITH_LOG("Invalid arg_len [%d]", arg_len);
    }

    CAN_Config_Tx_Data_Drv(proxy_dev_g[proxy_index].can_index, CAN_STID, cmd_id, arg_len);
    rlt = CAN_Send(proxy_dev_g[proxy_index].can_index, (u8 *)arg);
    if (HAL_OK != rlt) {
        ZSS_PROXY_LOGE("CAN[%d] send failed [%d]\r\n", proxy_dev_g[proxy_index].can_index, rlt);
        return COMMON_ERROR_ZMZ;
    }

    return SUCCESS_ZMZ;
}

zss_return_e Zss_Proxy_Send_Retry(proxy_index_e proxy_index, ZSS_CMD_ID_e cmd_id, void *arg, u8 retry_times)
{
    zss_return_e rlt = SUCCESS_ZMZ;

    for (u8 i = 0; i < retry_times; i++) {
        rlt = Zss_Proxy_Send(proxy_index, cmd_id, arg);
        if (SUCCESS_ZMZ == rlt) {
            return rlt;
        }

        if (i) {
            ZSS_PROXY_LOGI("retry: %d\r\n", i);
        }
    }
    
    ZSS_PROXY_LOGE("CAN[%d] send failed [%d]\r\n", proxy_dev_g[proxy_index].can_index, rlt);
    return COMMON_ERROR_ZMZ;
}

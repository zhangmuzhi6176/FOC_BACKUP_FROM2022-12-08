#include "zmz_can_drv_STM32F103.h"
#include "zmz_gpio_drv_STM32F103.h"

#include "zmz_proxy.h"
#include "zmz_uart_hal.h"

typedef struct can_dev {
    u32                             can_template_filter_id[CAN_TEMPLATE_FILTER_NUM];
    float                           baud;

    CAN_RxHeaderTypeDef             RxHeader;
    CAN_TxHeaderTypeDef             TxHeader;
    CAN_HandleTypeDef               instance;
    u32                             pTxMailbox;

    u8                              can_number;

    gpio_spec_t                     can_tx_io;
    gpio_spec_t                     can_rx_io;
    IRQn_Type                       irq_no;
    u32                             preempt_priority;
    u32                             sub_priority;

    u8                              Rx_buf[CAN_BUF_SIZE];
} can_dev_t;

static can_dev_t can_dev_g[] = {
    [CAN_I] = {
        .can_template_filter_id = {
            0x1,
            0x0,
            0x0,
            0x0,
        },

        .instance = {
            .Instance = CAN1,
            .Init.Prescaler = 4,
            .Init.Mode = CAN_MODE_NORMAL,
            .Init.SyncJumpWidth = CAN_SJW_1TQ,
            .Init.TimeSeg1 = CAN_BS1_4TQ,
            .Init.TimeSeg2 = CAN_BS2_4TQ,
            .Init.TimeTriggeredMode = DISABLE,
            .Init.AutoBusOff = ENABLE,
            .Init.AutoWakeUp = ENABLE,
            .Init.AutoRetransmission = ENABLE,
            .Init.ReceiveFifoLocked = DISABLE,
            .Init.TransmitFifoPriority = DISABLE,
        },

        .TxHeader = {
            .TransmitGlobalTime = DISABLE,
        },

        .can_number = 1,

        .can_tx_io = {
            .gpio_grp = A,
            .gpio_num = 12,
        },
        .can_rx_io = {
            .gpio_grp = A,
            .gpio_num = 11,
        },
        .irq_no = USB_LP_CAN1_RX0_IRQn,
        .preempt_priority = 2,
        .sub_priority = 0,
    }
};

static u32 _SyncJumpWidth_Reg_Data_2_Val(u32 reg_data)
{
    u32 val = 0;
    switch (reg_data) {
    case CAN_SJW_1TQ:
        val = 1;
        break;
    case CAN_SJW_2TQ:
        val = 2;
        break;
    case CAN_SJW_3TQ:
        val = 3;
        break;
    case CAN_SJW_4TQ:
        val = 4;
        break;
    default:
        ZSS_ASSERT_WITH_LOG("Invalid SyncJumpWidth reg_data [%d]", reg_data);
    }
    return val;
}

static u32 _TimeSeg1_Reg_Data_2_Val(u32 reg_data)
{
    u32 val = 0;
    switch (reg_data) {
    case CAN_BS1_1TQ:
        val = 1;
        break;
    case CAN_BS1_2TQ:
        val = 2;
        break;
    case CAN_BS1_3TQ:
        val = 3;
        break;
    case CAN_BS1_4TQ:
        val = 4;
        break;
    case CAN_BS1_5TQ:
        val = 5;
        break;
    case CAN_BS1_6TQ:
        val = 6;
        break;
    case CAN_BS1_7TQ:
        val = 7;
        break;
    case CAN_BS1_8TQ:
        val = 8;
        break;
    case CAN_BS1_9TQ:
        val = 9;
        break;
    case CAN_BS1_10TQ:
        val = 10;
        break;
    case CAN_BS1_11TQ:
        val = 11;
        break;
    case CAN_BS1_12TQ:
        val = 12;
        break;
    case CAN_BS1_13TQ:
        val = 13;
        break;
    case CAN_BS1_14TQ:
        val = 14;
        break;
    case CAN_BS1_15TQ:
        val = 15;
        break;
    case CAN_BS1_16TQ:
        val = 16;
        break;
    default:
        ZSS_ASSERT_WITH_LOG("Invalid TimeSeg1 reg_data [%d]", reg_data);
    }
    return val;
}

static u32 _TimeSeg2_Reg_Data_2_Val(u32 reg_data)
{
    u32 val = 0;
    switch (reg_data) {
    case CAN_BS2_1TQ:
        val = 1;
        break;
    case CAN_BS2_2TQ:
        val = 2;
        break;
    case CAN_BS2_3TQ:
        val = 3;
        break;
    case CAN_BS2_4TQ:
        val = 4;
        break;
    case CAN_BS2_5TQ:
        val = 5;
        break;
    case CAN_BS2_6TQ:
        val = 6;
        break;
    case CAN_BS2_7TQ:
        val = 7;
        break;
    case CAN_BS2_8TQ:
        val = 8;
        break;
    default:
        ZSS_ASSERT_WITH_LOG("Invalid TimeSeg2 reg_data [%d]", reg_data);
    }
    return val;
}

/*
@baud:  baud = F_clk / (Prescaler * (SyncJumpWidth + TimeSeg1 + TimeSeg2))
             = 36M / (4 * (1 + 4 + 4))
             = 1M

        ----------------|----------------------------|----------------------------
        SyncJumpWidth   |         TimeSeg1           |         TimeSeg2
                                                     ^
                                                Sampling point (is between TimeSeg1 and TimeSeg2, so TimeSeg1 should be slightly shorter than TimeSeg2 and they should NOT be too short)
 */
static float _CAN_Calc_Baud_MHz(can_index_e index)
{
    u32 clk_source_MHz, SyncJumpWidth, TimeSeg1, TimeSeg2 = 0;

    switch (can_dev_g[index].can_number) {
    case 1:
        clk_source_MHz = Stm32_Get_PCLK_1_MHz();
        break;
    case 2:
        clk_source_MHz = Stm32_Get_PCLK_1_MHz();
        break;
    default:
        ZSS_ASSERT_WITH_LOG("Invalid SyncJumpWidth can_number [%d]", can_dev_g[index].can_number);
    }

    SyncJumpWidth = _SyncJumpWidth_Reg_Data_2_Val(can_dev_g[index].instance.Init.SyncJumpWidth);
    TimeSeg1 = _TimeSeg1_Reg_Data_2_Val(can_dev_g[index].instance.Init.TimeSeg1);
    TimeSeg2 = _TimeSeg2_Reg_Data_2_Val(can_dev_g[index].instance.Init.TimeSeg2);
    ZSS_CAN_LOGI("CAN[%d] clk_source[%d]MHz, SyncJumpWidth[%d], TimeSeg1[%d], TimeSeg2[%d].\r\n", index, clk_source_MHz, SyncJumpWidth, TimeSeg1, TimeSeg2);

    return ((float)clk_source_MHz / (float)(can_dev_g[index].instance.Init.Prescaler * (SyncJumpWidth + TimeSeg1 + TimeSeg2)));
}

void CAN_Init_Drv(void)
{
    int rlt = HAL_OK;

    for (int i = 0; i < ZSS_ARRAY_SIZE(can_dev_g); i++) {
        rlt = HAL_CAN_Init(&(can_dev_g[i].instance));
        if (HAL_OK != rlt) {
            ZSS_ASSERT_WITH_LOG("HAL_CAN_Init failed [%d].\r\n", rlt);
        }
        can_dev_g[i].baud = _CAN_Calc_Baud_MHz((can_index_e)i);

        ZSS_CAN_LOGI("CAN[%d] is initialized at baud [%f]MHz.\r\n", i, can_dev_g[i].baud);
    }

    ZSS_CAN_LOGI("CAN_Init_Drv successful.\r\n");
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    for (u8 i = 0; i < ZSS_ARRAY_SIZE(can_dev_g); i++) {
        if (canHandle->Instance == can_dev_g[i].instance.Instance) {
            switch (can_dev_g[i].can_number) {
            case 1:
                __HAL_RCC_CAN1_CLK_ENABLE();
                break;
            default:
                ZSS_ASSERT_WITH_LOG("__HAL_RCC_CAN[%d]_CLK_ENABLE Unsupported", can_dev_g[i].can_number);
            }

            switch (can_dev_g[i].can_tx_io.gpio_grp) {
                CAN_TX_CASE_GPIO_INIT(A, can_dev_g[i].can_tx_io.gpio_num);
                CAN_TX_CASE_GPIO_INIT(B, can_dev_g[i].can_tx_io.gpio_num);
                CAN_TX_CASE_GPIO_INIT(C, can_dev_g[i].can_tx_io.gpio_num);
                CAN_TX_CASE_GPIO_INIT(D, can_dev_g[i].can_tx_io.gpio_num);
                CAN_TX_CASE_GPIO_INIT(E, can_dev_g[i].can_tx_io.gpio_num);
            }

            switch (can_dev_g[i].can_rx_io.gpio_grp) {
                CAN_RX_CASE_GPIO_INIT(A, can_dev_g[i].can_rx_io.gpio_num);
                CAN_RX_CASE_GPIO_INIT(B, can_dev_g[i].can_rx_io.gpio_num);
                CAN_RX_CASE_GPIO_INIT(C, can_dev_g[i].can_rx_io.gpio_num);
                CAN_RX_CASE_GPIO_INIT(D, can_dev_g[i].can_rx_io.gpio_num);
                CAN_RX_CASE_GPIO_INIT(E, can_dev_g[i].can_rx_io.gpio_num);
            }

            HAL_NVIC_SetPriority(can_dev_g[i].irq_no, can_dev_g[i].preempt_priority, can_dev_g[i].sub_priority);
            HAL_NVIC_EnableIRQ(can_dev_g[i].irq_no);
        }
    }
}

void CAN_Config_Tx_Data_Drv(can_index_e index, can_ID_type_e ID_type, u32 id, u8 data_len)
{
    can_dev_g[index].TxHeader.RTR = CAN_DATA;
    can_dev_g[index].TxHeader.DLC = data_len;

    if (CAN_STID == ID_type) {
        can_dev_g[index].TxHeader.StdId = id;
    } else if (CAN_EXID == ID_type) {
        can_dev_g[index].TxHeader.ExtId = id;
    } else {
        ZSS_ASSERT_WITH_LOG("Invalid ID_type [%d]", ID_type);
    }

}

HAL_StatusTypeDef CAN_Send(can_index_e index, u8 *tx_data)
{
    HAL_StatusTypeDef rlt = HAL_OK;

    rlt = HAL_CAN_AddTxMessage(&(can_dev_g[index].instance), &(can_dev_g[index].TxHeader), tx_data, &(can_dev_g[index].pTxMailbox));
    if (HAL_OK != rlt) {
        ZSS_CAN_LOGE("HAL_CAN_AddTxMessage failed [%d]\r\n", rlt);
    }

    ZSS_CAN_LOGI("CAN[%d], STID[0x%x] EXID[0x%x] Sending: %d %d %d %d %d %d %d %d\r\n", index, can_dev_g[index].TxHeader.StdId, can_dev_g[index].TxHeader.ExtId, 
                 tx_data[0],
                 tx_data[1],
                 tx_data[2],
                 tx_data[3],
                 tx_data[4],
                 tx_data[5],
                 tx_data[6],
                 tx_data[7]);

    return rlt;
}

/*
    @Filter mode specification
        I:      16 BIT LIST MODE
            * FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow each contains one whitelist ID as follows.
                -----------------------------------------------------------------
                |             [15:8]            |             [7:0]             |
                -----------------------------------------------------------------
                             STID[10:3]         |  STID[2:0]+RTR+IDE+EXID[17:15]


        II:     16 BIT MASK MODE
            * FilterIdHigh, FilterIdLow each contains one whitelist ID as follows.
                -----------------------------------------------------------------
                |             [15:8]            |             [7:0]             |
                -----------------------------------------------------------------
                             STID[10:3]         |  STID[2:0]+RTR+IDE+EXID[17:15]

            * FilterMaskIdHigh, FilterMaskIdLow each contains one 16-bit MASK, bit set indicates ‘must match with the ID’, bit unset indicates 'don't care', e.g:
                -----------------------------------------------------------------
                | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 0 |
                -----------------------------------------------------------------
                                    STID                    |
                means the remote STID passes the filter only if bit[0] and bit[6] matches with FilterMaskIdHigh or FilterMaskIdLow.
                NOTE: The 16 BIT MASK MODE can also filter undesired IDE value (by setting/unsetting bit[3]) or RTR value (by setting/unsetting bit[4]) of FilterMaskIdHigh / FilterMaskIdLow


        III:    32 BIT LIST MODE
            * (FilterIdHigh + FilterIdLow), (FilterMaskIdHigh + FilterMaskIdLow) each contains one whitelist ID as follows.
                ----------------------------------------------------------------------------------------------------------------------------------
                |             [31:24]            |            [23:16]            |             [15:8]            |             [7:0]             |
                ----------------------------------------------------------------------------------------------------------------------------------
                             STID[10:3]          |    STID[2:0]+EXID[17:13]      |           EXID[12:5]          |       EXID[4:0]+IDE+RTR+0
                                                                    FilterIdHigh + FilterIdLow
                                                                FilterMaskIdHigh + FilterMaskIdLow


        IV:     32 BIT MASK MODE
            * (FilterIdHigh + FilterIdLow) contains one whitelist ID as follows.
                ----------------------------------------------------------------------------------------------------------------------------------
                |             [31:24]            |            [23:16]            |             [15:8]            |             [7:0]             |
                ----------------------------------------------------------------------------------------------------------------------------------
                             STID[10:3]          |    STID[2:0]+EXID[17:13]      |           EXID[12:5]          |       EXID[4:0]+IDE+RTR+0
                                                                    FilterIdHigh + FilterIdLow

            * (FilterMaskIdHigh + FilterMaskIdLow) contains one 32-bit MASK, bit set indicates ‘must match with the ID’, bit unset indicates 'don't care', e.g:
                ---------------------------------------------------------------------------------------------------------------------------------
                | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
                ---------------------------------------------------------------------------------------------------------------------------------
                                    STID                    |                                  EXID                                 |
                means the remote STID passes the filter only if bit[0] and bit[6] matches with (FilterIdHigh + FilterIdLow)
                NOTE: The 32 BIT MASK MODE can also filter undesired IDE value (by setting/unsetting bit[3]) or RTR value (by setting/unsetting bit[2]) of (FilterMaskIdHigh + FilterMaskIdLow)
 */
CAN_FilterTypeDef CAN_filter_conf_template = {
    .FilterActivation = CAN_FILTER_ENABLE,
    .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterBank = 0,
    .SlaveStartFilterBank = 0,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterIdHigh = 0,
    .FilterIdLow = 0,
    .FilterMaskIdHigh = 0,
    .FilterMaskIdLow = 0,
};

HAL_StatusTypeDef CAN_Config_Rx_Drv(can_index_e index, CAN_FilterTypeDef CAN_filter_conf)
{
    HAL_StatusTypeDef rlt = HAL_OK;

    if (!memcmp(&CAN_filter_conf, &CAN_filter_conf_template, sizeof(CAN_FilterTypeDef))) {
        CAN_filter_conf.FilterIdHigh = can_dev_g[index].can_template_filter_id[0] << 5;
        CAN_filter_conf.FilterIdLow = can_dev_g[index].can_template_filter_id[1] << 5;
        CAN_filter_conf.FilterMaskIdHigh = can_dev_g[index].can_template_filter_id[2] << 5;
        CAN_filter_conf.FilterMaskIdLow = can_dev_g[index].can_template_filter_id[3] << 5;
    }

    rlt = HAL_CAN_ConfigFilter(&(can_dev_g[index].instance), &CAN_filter_conf);
    if (HAL_OK != rlt) {
        ZSS_ASSERT_WITH_LOG("HAL_CAN_ConfigFilter failed [%d].\r\n", rlt);
    }

    rlt = HAL_CAN_Start(&(can_dev_g[index].instance));
    if (HAL_OK != rlt) {
        ZSS_ASSERT_WITH_LOG("HAL_CAN_ConfigFilter failed [%d].\r\n", rlt);
    }

    rlt = HAL_CAN_ActivateNotification(&(can_dev_g[index].instance), CAN_IT_RX_FIFO0_MSG_PENDING);
    if (HAL_OK != rlt) {
        ZSS_ASSERT_WITH_LOG("HAL_CAN_ConfigFilter failed [%d].\r\n", rlt);
    }

    ZSS_CAN_LOGI("CAN Config successful.\r\n");
    return rlt;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef rlt = HAL_OK;
    proxy_dev_t *proxy_dev = NULL;

    for (u8 i = 0; i < ZSS_ARRAY_SIZE(can_dev_g); i++) {
        if (hcan->Instance == can_dev_g[i].instance.Instance) {
            rlt = HAL_CAN_GetRxMessage(&(can_dev_g[i].instance), CAN_FILTER_FIFO0, &(can_dev_g[i].RxHeader), can_dev_g[i].Rx_buf);
            if (HAL_OK != rlt) {
                ZSS_CAN_LOGE("HAL_CAN_GetRxMessage failed [%d]\r\n", rlt);
            }

            proxy_dev = Get_Proxy_By_CAN_idx((can_index_e)i);
            if (proxy_dev) {
                proxy_dev->Proxy_Response_Func(can_dev_g[i].RxHeader.StdId, can_dev_g[i].Rx_buf);
            }
        }
    }
}


/* --------------------------------------------------------------- CAN Interrupt callbacks --------------------------------------------------------------- */

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&(can_dev_g[CAN_I].instance));
}

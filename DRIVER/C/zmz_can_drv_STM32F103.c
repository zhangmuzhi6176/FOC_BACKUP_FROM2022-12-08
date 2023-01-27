#include "zmz_uart_hal.h"

#include "zmz_can_drv_STM32F103.h"
#include "zmz_gpio_drv_STM32F103.h"

typedef struct can_dev {
    CAN_RxHeaderTypeDef RxHeader;
    CAN_TxHeaderTypeDef TxHeader;
    CAN_HandleTypeDef instance;
    u32 pTxMailbox;

    u8 can_number;

    gpio_spec_t can_tx_io;
    gpio_spec_t can_rx_io;
    IRQn_Type irq_no;
    u32 preempt_priority;
    u32 sub_priority;
} can_dev_t;

can_dev_t can_dev_g[] = {
    [0] = {
        .instance = {
            .Instance = CAN1,
            .Init.Prescaler = 5,
            .Init.Mode = CAN_MODE_NORMAL,
            .Init.SyncJumpWidth = CAN_SJW_1TQ,
            .Init.TimeSeg1 = CAN_BS1_3TQ,
            .Init.TimeSeg2 = CAN_BS1_5TQ,
            .Init.TimeTriggeredMode = DISABLE,
            .Init.AutoBusOff = ENABLE,
            .Init.AutoWakeUp = ENABLE,
            .Init.AutoRetransmission = ENABLE,
            .Init.ReceiveFifoLocked = DISABLE,
            .Init.TransmitFifoPriority = DISABLE,
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





CAN_HandleTypeDef hcan;

u8 Can_RxData[8];
u8 Can_TxData[8] = {0, 1, 2, 3, 4, 5, 6, 7};
u32 pTxMailbox = 0;
CAN_RxHeaderTypeDef CanRx;
CAN_TxHeaderTypeDef CanTx;


void MX_CAN_Init(void)
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 5;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
    hcan.Init.TimeSeg2 = CAN_BS1_5TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_OK == HAL_CAN_Init(&hcan)) {
        ZSS_CAN_LOGI("CAN initialization successful.\r\n");
    } else {
        ZSS_ASSERT_WITH_LOG("CAN initialization failed.\r\n");
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(can_dev_g); i++) {
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





HAL_StatusTypeDef Can_Config(void)
{

    CanTx.DLC = 8;
    CanTx.ExtId = CAN_ID;
    CanTx.IDE = CAN_ID_EXT;
    CanTx.RTR = CAN_RTR_DATA;
    CanTx.StdId = 0x00;
    CanTx.TransmitGlobalTime = DISABLE;


    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;                                            //筛选器使能
    sFilterConfig.FilterBank = 0;                                                                  //筛选器0
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;                                         //指定将分配给过滤器的FIFO(0或1U)。
    sFilterConfig.FilterIdHigh = (((CAN_FILTER_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF0000) >> 16; // ID的高16位
    sFilterConfig.FilterIdLow = ((CAN_FILTER_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;              // ID的低16位，只接收扩展帧模式、数据帧
    sFilterConfig.FilterMaskIdHigh = 0xFFFF;                                                       // FilterMask高低字节数据中位为1时代表必须与ID该位一致，0xFFFFFFFF代表接收筛选必须与ID一致才通过
    sFilterConfig.FilterMaskIdLow = 0xFFFF;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }



    ZSS_CAN_LOGI("CAN Config successful.\r\n");
    return HAL_OK;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CanRx, Can_RxData) == HAL_OK) {
        ZSS_CAN_LOGI("Can_RxData[0]:[%4d]    RX_ID[%x]      I am [%x]\r\n", Can_RxData[0], CanRx.ExtId, CAN_ID);
    }
}


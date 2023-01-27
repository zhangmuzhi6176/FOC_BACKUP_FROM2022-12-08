#include "zmz_can_drv_STM32F103.h"
#include "zmz_uart_hal.h"

typedef struct can_dev {
    CAN_RxHeaderTypeDef CanRx;
    CAN_TxHeaderTypeDef CanTx;
} can_dev_t;





























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
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        ZSS_ASSERT_WITH_LOG("CAN initialization failed\r\n");
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1) {

        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


        HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

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



    ZSS_CAN_LOGI("CAN initialization successful\r\n");
    return HAL_OK;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CanRx, Can_RxData) == HAL_OK) {
        ZSS_CAN_LOGI("Can_RxData[0]:[%4d]    RX_ID[%x]\r\n", Can_RxData[0], CanRx.ExtId);
    }
}


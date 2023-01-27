#ifndef _ZMZ_CAN_DRV_STM32F103_H
#define _ZMZ_CAN_DRV_STM32F103_H

#include "zmz_system_hardware.h"
#include "stm32f1xx_hal.h"


#define ZSS_CAN_LOGD(KEY, format, ...) ZSS_LOGD("CAN", KEY, format, ##__VA_ARGS__)
#define ZSS_CAN_LOGI(format, ...) ZSS_LOGI("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGW(format, ...) ZSS_LOGW("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGE(format, ...) ZSS_LOGE("CAN", format, ##__VA_ARGS__)
#define ZSS_CAN_LOGF(format, ...) ZSS_LOGF("CAN", format, ##__VA_ARGS__)








#define CAN_ID ((uint32_t)0x1315)
#define CAN_FILTER_ID ((uint32_t)0x1314)




extern CAN_HandleTypeDef hcan;

extern u8 Can_RxData[8];
extern u8 Can_TxData[8];
extern u32 pTxMailbox;
extern CAN_RxHeaderTypeDef CanRx;
extern CAN_TxHeaderTypeDef CanTx;



void MX_CAN_Init(void);
HAL_StatusTypeDef Can_Config(void);



#endif

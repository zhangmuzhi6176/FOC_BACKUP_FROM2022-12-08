#ifndef __can_H
#define __can_H

#include "zmz_system_hardware.h"
#include "stm32f1xx_hal.h"

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



#endif /*__ can_H */


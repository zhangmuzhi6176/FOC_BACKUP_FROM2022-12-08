#ifndef _MT6701_H
#define _MT6701_H

#include "zmz_uart_hal.h"
#include "zmz_system_hardware.h"

#define MT_ADDR 0X06

#define MT_ANGLE_ADDR_H 0X03
#define MT_ANGLE_ADDR_L 0X04

#define MT_PRECISION_BITS 14

typedef enum MT_Idx {
    ENC_NO_1 = 0,
    ENC_NO_2,
} MT_Idx_e;

int MT_Write_Byte(u8 index, u8 reg, u8 data);
u8 MT_Read_Byte(u8 index, u8 reg);
int MT_Write_Bytes(u8 index, u8 reg, u8 len, u8 *buf);
int MT_Read_Bytes(u8 index, u8 reg, u8 len, u8 *buf);
void MT_Init(void);
float MT_Get_ANGLE(u8 index);

#endif

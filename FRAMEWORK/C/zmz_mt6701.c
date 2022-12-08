#include "zmz_mt6701.h"
#include "zmz_iic_soft_hal.h"

int MT_Write_Byte(u8 index, u8 reg, u8 data)
{
    return IIC_Common_Write_Byte(index, reg, data);
}

u8 MT_Read_Byte(u8 index, u8 reg)
{
    return IIC_Common_Read_Byte(index, reg);
}

int MT_Write_Bytes(u8 index, u8 reg, u8 len, u8 *buf)
{
    return IIC_Common_Write_Bytes(index, reg, len, buf);
}

int MT_Read_Bytes(u8 index, u8 reg, u8 len, u8 *buf)
{
    return IIC_Common_Read_Bytes(index, reg, len, buf);
}

void MT_Init(void)
{
    Set_IIC_Dev_Addr_Hal(ENC_NO_1, MT_ADDR);
    Set_IIC_Dev_Addr_Hal(ENC_NO_2, MT_ADDR);
}

float MT_Get_ANGLE(u8 index)
{
    u8 buf[2];
    short raw;
    float angle;
    buf[0] = MT_Read_Byte(index, MT_ANGLE_ADDR_H);
    buf[1] = MT_Read_Byte(index, MT_ANGLE_ADDR_L);
    raw = ((u16)buf[0] << 6) | ((u16)buf[1] >> 2);
    angle = (float)raw / (float)16384 * (float)360;
    return angle;
}

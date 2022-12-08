#include "zmz_delay.h"
#include "zmz_iic_soft_hal.h"
#include "zmz_iic_soft_drv_STM32F103.h"
#include "zmz_system_hardware.h"

static void _Sda_Out(u8 index, char value)
{
    switch (Get_Sda_IOgrp(index)) {
    case A:
        PAout(Get_Sda_IOnum(index)) = value;
        break;
    case B:
        PBout(Get_Sda_IOnum(index)) = value;
        break;
    case C:
        PCout(Get_Sda_IOnum(index)) = value;
        break;
    case D:
        PDout(Get_Sda_IOnum(index)) = value;
        break;
    case E:
        PEout(Get_Sda_IOnum(index)) = value;
        break;
    }
}

static char _Sda_In(u8 index)
{
    switch (Get_Sda_IOgrp(index)) {
    case A:
        return PAin(Get_Sda_IOnum(index));
    case B:
        return PBin(Get_Sda_IOnum(index));
    case C:
        return PCin(Get_Sda_IOnum(index));
    case D:
        return PDin(Get_Sda_IOnum(index));
    case E:
        return PEin(Get_Sda_IOnum(index));
    }
    return 0;
}

static void _Scl_Out(u8 index, char value)
{
    switch (Get_Scl_IOgrp(index)) {
    case A:
        PAout(Get_Scl_IOnum(index)) = value;
        break;
    case B:
        PBout(Get_Scl_IOnum(index)) = value;
        break;
    case C:
        PCout(Get_Scl_IOnum(index)) = value;
        break;
    case D:
        PDout(Get_Scl_IOnum(index)) = value;
        break;
    case E:
        PEout(Get_Scl_IOnum(index)) = value;
        break;
    }
}

void Set_IIC_Dev_Addr_Hal(u8 index, u8 addr)
{
    Set_IIC_Dev_Addr_Drv(index, addr);
}

u8 Get_IIC_Dev_Addr_Hal(u8 index)
{
    return Get_IIC_Dev_Addr_Drv(index);
}

void IIC_Init_Hal(void)
{
    IIC_Init_Drv();
}

void IIC_Start(u8 index)
{
    Set_Sda_Out(index);
    _Sda_Out(index, IIC_SIG_HIGH);
    _Scl_Out(index, IIC_SIG_HIGH);
    delay_us(4);
    _Sda_Out(index, IIC_SIG_LOW); /* START: when SCL is high, SDA changes form high to low */
    delay_us(4);
    _Scl_Out(index, IIC_SIG_LOW);
}

void IIC_Stop(u8 index)
{
    Set_Sda_Out(index);
    _Scl_Out(index, IIC_SIG_LOW);
    _Sda_Out(index, IIC_SIG_LOW); /* STOP: when SCL is high, SDA change form low to high */
    delay_us(4);
    _Scl_Out(index, IIC_SIG_HIGH);
    _Sda_Out(index, IIC_SIG_HIGH);
    delay_us(4);
}

u8 IIC_Wait_Ack(u8 index)
{
    u8 ucErrTime = 0;
    Set_Sda_In(index);
    _Sda_Out(index, IIC_SIG_HIGH);
    delay_us(1);
    _Scl_Out(index, IIC_SIG_HIGH);
    delay_us(1);
    while (_Sda_In(index)) {
        ucErrTime++;
        if (ucErrTime > 250) {
            IIC_Stop(index);
            return 1;
        }
    }
    _Scl_Out(index, IIC_SIG_LOW);
    return 0;
}

void IIC_Ack(u8 index)
{
    _Scl_Out(index, IIC_SIG_LOW);
    Set_Sda_Out(index);
    _Sda_Out(index, IIC_SIG_LOW);
    delay_us(2);
    _Scl_Out(index, IIC_SIG_HIGH);
    delay_us(2);
    _Scl_Out(index, IIC_SIG_LOW);
}

void IIC_NAck(u8 index)
{
    _Scl_Out(index, IIC_SIG_LOW);
    Set_Sda_Out(index);
    _Sda_Out(index, IIC_SIG_HIGH);
    delay_us(2);
    _Scl_Out(index, IIC_SIG_HIGH);
    delay_us(2);
    _Scl_Out(index, IIC_SIG_LOW);
}

void IIC_Send_Byte(u8 index, u8 txd)
{
    u8 t;
    Set_Sda_Out(index);
    _Scl_Out(index, IIC_SIG_LOW);
    for (t = 0; t < 8; t++) {
        _Sda_Out(index, ((txd & 0x80) >> 7)); /* Send data bit by bit */
        txd <<= 1;
        delay_us(2);
        _Scl_Out(index, IIC_SIG_HIGH);
        delay_us(2);
        _Scl_Out(index, IIC_SIG_LOW);
        delay_us(2);
    }
}

u8 IIC_Read_Byte(u8 index, u8 ack)
{
    u8 i, receive = 0;
    Set_Sda_In(index);
    for (i = 0; i < 8; i++) {
        _Scl_Out(index, IIC_SIG_LOW);
        delay_us(2);
        _Scl_Out(index, IIC_SIG_HIGH);
        receive <<= 1;
        if (_Sda_In(index))
            receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck(index);
    else
        IIC_Ack(index);
    return receive;
}

int IIC_Common_Write_Byte(u8 index, u8 start_reg, u8 data)
{
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_WRITE);
    if (IIC_Wait_Ack(index)) {
        IIC_Stop(index);
        return TIME_OUT_ZMZ;
    }
    IIC_Send_Byte(index, start_reg);
    IIC_Wait_Ack(index);
    IIC_Send_Byte(index, data);
    if (IIC_Wait_Ack(index)) {
        IIC_Stop(index);
        return TIME_OUT_ZMZ;
    }
    IIC_Stop(index);
    return SUCCESS_ZMZ;
}

u8 IIC_Common_Read_Byte(u8 index, u8 start_reg)
{
    u8 res;
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_WRITE);
    IIC_Wait_Ack(index);
    IIC_Send_Byte(index, start_reg);
    IIC_Wait_Ack(index);
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_READ);
    IIC_Wait_Ack(index);
    res = IIC_Read_Byte(index, SEND_NACK);
    IIC_Stop(index);
    return res;
}

int IIC_Common_Write_Bytes(u8 index, u8 start_reg, u8 len, u8 *buf)
{
    u8 i;
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_WRITE);
    if (IIC_Wait_Ack(index)) {
        IIC_Stop(index);
        return TIME_OUT_ZMZ;
    }
    IIC_Send_Byte(index, start_reg);
    IIC_Wait_Ack(index);
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(index, buf[i]);
        if (IIC_Wait_Ack(index)) {
            IIC_Stop(index);
            return TIME_OUT_ZMZ;
        }
    }
    IIC_Stop(index);
    return SUCCESS_ZMZ;
}

int IIC_Common_Read_Bytes(u8 index, u8 start_reg, u8 len, u8 *buf)
{
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_WRITE);
    if (IIC_Wait_Ack(index)) {
        IIC_Stop(index);
        return TIME_OUT_ZMZ;
    }
    IIC_Send_Byte(index, start_reg);
    IIC_Wait_Ack(index);
    IIC_Start(index);
    IIC_Send_Byte(index, (Get_IIC_Dev_Addr_Hal(index) << 1) | IIC_ADDR_READ);
    IIC_Wait_Ack(index);
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(index, SEND_NACK);
        else
            *buf = IIC_Read_Byte(index, SEND_ACK);
        len--;
        buf++;
    }
    IIC_Stop(index);
    return SUCCESS_ZMZ;
}

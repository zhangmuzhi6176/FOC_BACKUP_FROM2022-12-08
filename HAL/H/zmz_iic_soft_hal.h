#ifndef _ZMZ_IIC_SOFT_HAL_H
#define _ZMZ_IIC_SOFT_HAL_H
#include "zmz_system_hardware.h"

#define IIC_SIG_HIGH 1
#define IIC_SIG_LOW 0

#define IIC_ADDR_WRITE 0
#define IIC_ADDR_READ 1

#define SEND_NACK 0
#define SEND_ACK 1

void Set_IIC_Dev_Addr_Hal(u8 index, u8 addr);
u8 Get_IIC_Dev_Addr_Hal(u8 index);

void IIC_Init_Hal(void);

void IIC_Start(u8 index);
void IIC_Stop(u8 index);
/* @return:     1, ack failed
 *              0, ack successful
 */
u8 IIC_Wait_Ack(u8 index);
void IIC_Ack(u8 index);
void IIC_NAck(u8 index);
void IIC_Send_Byte(u8 index, u8 txd);
/* @param:      ack:    1, send ack
 *                      0, do not send ack
 */
u8 IIC_Read_Byte(u8 index, u8 ack);

int IIC_Common_Write_Byte(u8 index, u8 start_reg, u8 data);
u8 IIC_Common_Read_Byte(u8 index, u8 start_reg);
int IIC_Common_Write_Bytes(u8 index, u8 start_reg, u8 len, u8 *buf);
int IIC_Common_Read_Bytes(u8 index, u8 start_reg, u8 len, u8 *buf);

#endif

#include "zmz_iic_soft_drv_STM32F103.h"

typedef struct iic_dev {
    gpio_spec_t sda_io;
    gpio_spec_t scl_io;

    u8 device_addr;
} iic_dev_t;

static iic_dev_t iic_dev_g[] = {
    [0] = {
        .sda_io = {
            .gpio_grp = B,
            .gpio_num = 7,
        },
        .scl_io = {
            .gpio_grp = B,
            .gpio_num = 6,
        },
    },
    [1] = {
        .sda_io = {
            .gpio_grp = B,
            .gpio_num = 9,
        },
        .scl_io = {
            .gpio_grp = B,
            .gpio_num = 8,
        },
    },
};

void Set_Sda_In(u8 index)
{
    switch (iic_dev_g[index].sda_io.gpio_grp) {
    IIC_CASE_SET_GPIO_CR(A, iic_dev_g[index].sda_io.gpio_num, 8);
    IIC_CASE_SET_GPIO_CR(B, iic_dev_g[index].sda_io.gpio_num, 8);
    IIC_CASE_SET_GPIO_CR(C, iic_dev_g[index].sda_io.gpio_num, 8);
    IIC_CASE_SET_GPIO_CR(D, iic_dev_g[index].sda_io.gpio_num, 8);
    IIC_CASE_SET_GPIO_CR(E, iic_dev_g[index].sda_io.gpio_num, 8);
    }
}

void Set_Sda_Out(u8 index)
{
    switch (iic_dev_g[index].sda_io.gpio_grp) {
    IIC_CASE_SET_GPIO_CR(A, iic_dev_g[index].sda_io.gpio_num, 3);
    IIC_CASE_SET_GPIO_CR(B, iic_dev_g[index].sda_io.gpio_num, 3);
    IIC_CASE_SET_GPIO_CR(C, iic_dev_g[index].sda_io.gpio_num, 3);
    IIC_CASE_SET_GPIO_CR(D, iic_dev_g[index].sda_io.gpio_num, 3);
    IIC_CASE_SET_GPIO_CR(E, iic_dev_g[index].sda_io.gpio_num, 3);
    }
}

u8 Get_Sda_IOgrp(u8 index)
{
    return iic_dev_g[index].sda_io.gpio_grp;
}

u8 Get_Sda_IOnum(u8 index)
{
    return iic_dev_g[index].sda_io.gpio_num;
}

u8 Get_Scl_IOgrp(u8 index)
{
    return iic_dev_g[index].scl_io.gpio_grp;
}

u8 Get_Scl_IOnum(u8 index)
{
    return iic_dev_g[index].scl_io.gpio_num;
}

void Set_IIC_Dev_Addr_Drv(u8 index, u8 addr)
{
    iic_dev_g[index].device_addr = addr;
}

u8 Get_IIC_Dev_Addr_Drv(u8 index)
{
    return iic_dev_g[index].device_addr;
}

void IIC_Init_Drv(void)
{
    for (u8 index = 0; index < ZSS_ARRAY_SIZE(iic_dev_g); index++) {
        switch (iic_dev_g[index].sda_io.gpio_grp) {
            IIC_CASE_GPIO_INIT_LOG(A, iic_dev_g[index].sda_io.gpio_num, "IIC_SDA");
            IIC_CASE_GPIO_INIT_LOG(B, iic_dev_g[index].sda_io.gpio_num, "IIC_SDA");
            IIC_CASE_GPIO_INIT_LOG(C, iic_dev_g[index].sda_io.gpio_num, "IIC_SDA");
            IIC_CASE_GPIO_INIT_LOG(D, iic_dev_g[index].sda_io.gpio_num, "IIC_SDA");
            IIC_CASE_GPIO_INIT_LOG(E, iic_dev_g[index].sda_io.gpio_num, "IIC_SDA");
        }
        switch (iic_dev_g[index].scl_io.gpio_grp) {
            IIC_CASE_GPIO_INIT_LOG(A, iic_dev_g[index].scl_io.gpio_num, "IIC_SCL");
            IIC_CASE_GPIO_INIT_LOG(B, iic_dev_g[index].scl_io.gpio_num, "IIC_SCL");
            IIC_CASE_GPIO_INIT_LOG(C, iic_dev_g[index].scl_io.gpio_num, "IIC_SCL");
            IIC_CASE_GPIO_INIT_LOG(D, iic_dev_g[index].scl_io.gpio_num, "IIC_SCL");
            IIC_CASE_GPIO_INIT_LOG(E, iic_dev_g[index].scl_io.gpio_num, "IIC_SCL");
        }
        ZSS_IIC_SOFT_LOGI("Device addr of IIC[%d]: [%d].\r\n", index, iic_dev_g[index].device_addr)
    }
}

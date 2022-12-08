#ifndef _ZMZ_SYSTEM_HARDWARE_H
#define _ZMZ_SYSTEM_HARDWARE_H

#include "zmz_system.h"
#include "stm32f1xx.h"

#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Offset 12
#define GPIOA_IDR_Offset 8

#define GPIOA_ODR_Addr (GPIOA_BASE + GPIOA_ODR_Offset) // 0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + GPIOA_ODR_Offset) // 0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + GPIOA_ODR_Offset) // 0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + GPIOA_ODR_Offset) // 0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + GPIOA_ODR_Offset) // 0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + GPIOA_ODR_Offset) // 0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + GPIOA_ODR_Offset) // 0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + GPIOA_IDR_Offset) // 0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + GPIOA_IDR_Offset) // 0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + GPIOA_IDR_Offset) // 0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + GPIOA_IDR_Offset) // 0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + GPIOA_IDR_Offset) // 0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + GPIOA_IDR_Offset) // 0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + GPIOA_IDR_Offset) // 0x40011E08

/* n MUST BE SMALLER THAN 16 !!! */
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)
/* n MUST BE SMALLER THAN 16 !!! */

#define ZSS_SHUT_DOWN_IT __set_PRIMASK(1)
#define ZSS_TURN_UP_IT __set_PRIMASK(0)

#define MAIN_FREQ 72 /* Hz */

void Stm32_Clock_Init(u32 PLL);
u16 Stm32_Get_Clock_MHz(void);

#endif

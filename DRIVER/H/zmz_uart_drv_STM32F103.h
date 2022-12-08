#ifndef _UART_H
#define _UART_H
#include "zmz_system_hardware.h"
#include "stdio.h"

#define UART_RX_BUFFERSIZE 200

#define CR_RECEIVED 14
#define LF_RECEIVED 15

#define LOG_D_KEY_MAX 30
#define DBG_INT_MAX 10

#define ZSS_UART_LOGD(KEY, format, ...) ZSS_LOGD("UART", KEY, format, ##__VA_ARGS__)
#define ZSS_UART_LOGI(format, ...) ZSS_LOGI("UART", format, ##__VA_ARGS__)
#define ZSS_UART_LOGW(format, ...) ZSS_LOGW("UART", format, ##__VA_ARGS__)
#define ZSS_UART_LOGE(format, ...) ZSS_LOGE("UART", format, ##__VA_ARGS__)
#define ZSS_UART_LOGF(format, ...) ZSS_LOGF("UART", format, ##__VA_ARGS__)

#define UART_1 0

typedef enum uart_receive_status {
    UART_RECEIVE_INCOMPLETED = 1,
    UART_RECEIVE_ALMOST_COMPLETED,
    UART_RECEIVE_COMPLETED,
} uart_receive_status_e;

#define UART_TX_CASE_GPIO_INIT(CASE, PIN)                                             \
    {                                                                                 \
        CASE_GPIO_INIT(CASE, PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH) \
    }

#define UART_RX_CASE_GPIO_INIT(CASE, PIN)                                                \
    {                                                                                    \
        CASE_GPIO_INIT(CASE, PIN, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH) \
    }

void Uart_Init(void);
u16 Get_Uart_Receive_len(u8 index);
uart_receive_status_e Get_Uart_Receive_Status(u8 index);
void Set_Uart_Receive_Status(u8 index, uart_receive_status_e flag);
bool Get_Uart_LOG_D_Status(u8 index);
void Set_Uart_LOG_D_Status(u8 index, bool status);
bool Match_Uart_LOG_D_KEY(u8 index, const char *key);
void Set_Uart_LOG_D_KEY(u8 index, const char *key);
void Set_Uart_DBG_INT(u8 index, int val, u8 dbg_int_idx);
int Get_Uart_DBG_INT(u8 index, u8 dbg_int_idx);
void Clean_Uart_DBG_INT(u8 index);

#endif

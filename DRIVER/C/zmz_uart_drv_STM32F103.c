#include "zmz_uart_hal.h"
#include "zmz_uart_drv_STM32F103.h"
#include "zmz_gpio_drv_STM32F103.h"
#include "zmz_system_hardware.h"

#include <stdlib.h>

struct __FILE {
    int handle;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0)
        ;
    USART1->DR = (u8)ch;
    return ch;
}

/* 防止进入半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

typedef struct uart {
    UART_HandleTypeDef uart_handler;
    USART_TypeDef *instance;

    u8 uart_index;
    gpio_spec_t uart_tx_io;
    gpio_spec_t uart_rx_io;
    IRQn_Type irq_no;
    u32 preempt_priority;
    u32 sub_priority;

    u32 baud_rate;
    u32 word_length;
    u32 stop_bits;
    u32 parity;
    u32 hardware_flow_ctrl;
    u32 mode;

    bool is_LOG_D_ON;
    char key_LOG_D[LOG_D_KEY_MAX];

    int dbg_int[DBG_INT_MAX];

    /* bit15:       接收到0x0a则为1，否则为0
    bit14:          接收到0x0d则为1，否则为0
    bit13~0:        接收到的有效字节数 */
    u16 rx_status;
    u8 rx_buffer[UART_RX_BUFFERSIZE];
} uart_t;

static uart_t uart_dev_g[] = {
    [0] = {
        .instance = USART1,
        .uart_index = 1,
        .uart_tx_io = {
            .gpio_grp = A,
            .gpio_num = 9,
        },
        .uart_rx_io = {
            .gpio_grp = A,
            .gpio_num = 10,
        },
        .irq_no = USART1_IRQn,
        .preempt_priority = 0,
        .sub_priority = 0,
        .baud_rate = 921600,
        .word_length = UART_WORDLENGTH_8B,
        .stop_bits = UART_STOPBITS_1,
        .parity = UART_PARITY_NONE,
        .hardware_flow_ctrl = UART_HWCONTROL_NONE,
        .mode = UART_MODE_TX_RX,
        .is_LOG_D_ON = false,
    },
};

void Uart_Init(void)
{
    for (int i = 0; i < ZSS_ARRAY_SIZE(uart_dev_g); i++) {
        uart_dev_g[i].uart_handler.Instance = uart_dev_g[i].instance;
        uart_dev_g[i].uart_handler.Init.BaudRate = uart_dev_g[i].baud_rate;
        uart_dev_g[i].uart_handler.Init.WordLength = uart_dev_g[i].word_length;
        uart_dev_g[i].uart_handler.Init.StopBits = uart_dev_g[i].stop_bits;
        uart_dev_g[i].uart_handler.Init.Parity = uart_dev_g[i].parity;
        uart_dev_g[i].uart_handler.Init.HwFlowCtl = uart_dev_g[i].hardware_flow_ctrl;
        uart_dev_g[i].uart_handler.Init.Mode = uart_dev_g[i].mode;
        memset(&(uart_dev_g[i].key_LOG_D[0]), 0, LOG_D_KEY_MAX);
        HAL_UART_Init(&(uart_dev_g[i].uart_handler));
        HAL_UART_Receive_IT(&(uart_dev_g[i].uart_handler), (u8 *) & (uart_dev_g[i].rx_buffer[0]), 1);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(uart_dev_g); i++) {
        if (huart->Instance == uart_dev_g[i].instance) {
            switch (uart_dev_g[i].uart_index) {
            case 1:
                __HAL_RCC_USART1_CLK_ENABLE();
                break;
            case 2:
                __HAL_RCC_USART2_CLK_ENABLE();
                break;
            case 3:
                __HAL_RCC_USART3_CLK_ENABLE();
                break;
            default:
                break;
            }

            switch (uart_dev_g[i].uart_tx_io.gpio_grp) {
                UART_TX_CASE_GPIO_INIT(A, uart_dev_g[i].uart_tx_io.gpio_num);
                UART_TX_CASE_GPIO_INIT(B, uart_dev_g[i].uart_tx_io.gpio_num);
                UART_TX_CASE_GPIO_INIT(C, uart_dev_g[i].uart_tx_io.gpio_num);
                UART_TX_CASE_GPIO_INIT(D, uart_dev_g[i].uart_tx_io.gpio_num);
                UART_TX_CASE_GPIO_INIT(E, uart_dev_g[i].uart_tx_io.gpio_num);
            }

            switch (uart_dev_g[i].uart_rx_io.gpio_grp) {
                UART_RX_CASE_GPIO_INIT(A, uart_dev_g[i].uart_rx_io.gpio_num);
                UART_RX_CASE_GPIO_INIT(B, uart_dev_g[i].uart_rx_io.gpio_num);
                UART_RX_CASE_GPIO_INIT(C, uart_dev_g[i].uart_rx_io.gpio_num);
                UART_RX_CASE_GPIO_INIT(D, uart_dev_g[i].uart_rx_io.gpio_num);
                UART_RX_CASE_GPIO_INIT(E, uart_dev_g[i].uart_rx_io.gpio_num);
            }

            HAL_NVIC_SetPriority(uart_dev_g[i].irq_no, uart_dev_g[i].preempt_priority, uart_dev_g[i].sub_priority);
            HAL_NVIC_EnableIRQ(uart_dev_g[i].irq_no);
        }
    }
}

u16 Get_Uart_Receive_len(u8 index)
{
    u16 len = uart_dev_g[index].rx_status & 0x3fff;
    return len;
}

uart_receive_status_e Get_Uart_Receive_Status(u8 index)
{
    if (uart_dev_g[index].rx_status & ((u32)1 << LF_RECEIVED)) {
        return UART_RECEIVE_COMPLETED;
    } else if (uart_dev_g[index].rx_status & ((u32)1 << CR_RECEIVED)) {
        return UART_RECEIVE_ALMOST_COMPLETED;
    } else {
        return UART_RECEIVE_INCOMPLETED;
    }
}

void Set_Uart_Receive_Status(u8 index, uart_receive_status_e flag)
{
    switch (flag) {
    case UART_RECEIVE_INCOMPLETED:
        uart_dev_g[index].rx_status = 0;
        break;
    case UART_RECEIVE_ALMOST_COMPLETED:
        uart_dev_g[index].rx_status |= ((u32)1 << CR_RECEIVED);
        break;
    case UART_RECEIVE_COMPLETED:
        uart_dev_g[index].rx_status |= ((u32)1 << LF_RECEIVED);
        break;
    }
}

static void _Uart_Receive(uart_t *uart_dev)
{
    u8 received_byte = 0;
    u8 index = ZSS_INDEX_IN_ARRAY(uart_dev_g, *uart_dev);
    received_byte = uart_dev->instance->DR;
    if (UART_RECEIVE_COMPLETED != Get_Uart_Receive_Status(index)) {
        if (UART_RECEIVE_ALMOST_COMPLETED != Get_Uart_Receive_Status(index)) {
            if ('\r' != received_byte) {
                uart_dev->rx_buffer[Get_Uart_Receive_len(index)] = received_byte;
                uart_dev->rx_status++;
                if (uart_dev->rx_status > (UART_RX_BUFFERSIZE - 1)) {
                    Set_Uart_Receive_Status(index, UART_RECEIVE_INCOMPLETED);
                }
            } else {
                Set_Uart_Receive_Status(index, UART_RECEIVE_ALMOST_COMPLETED);
            }
        } else {
            if ('\n' == received_byte) {
                Set_Uart_Receive_Status(index, UART_RECEIVE_COMPLETED);
            } else {
                Set_Uart_Receive_Status(index, UART_RECEIVE_INCOMPLETED);
            }
        }
    }
}

static void _Uart_CMD_DRV(uart_t *uart_dev)
{
    u8 index = ZSS_INDEX_IN_ARRAY(uart_dev_g, *uart_dev);
    u8 len = Get_Uart_Receive_len(index) + 1;
    char *cmd = (char *)malloc(len);
    if (UART_RECEIVE_COMPLETED == Get_Uart_Receive_Status(index)) {
        snprintf(cmd, len, "%s", &uart_dev_g[index].rx_buffer[0]);
        ZSS_UART_LOGI("WHOLE CMD is: [%s], CMD LENGTH: [%d]\r\n", cmd, len);
        Uart_Cmd(index, cmd);

        Set_Uart_Receive_Status(index, UART_RECEIVE_INCOMPLETED);
    }
    free(cmd);
}

void USART1_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&(uart_dev_g[UART_1].uart_handler), UART_FLAG_RXNE) != RESET)) {
        _Uart_Receive(&uart_dev_g[UART_1]);
    }
    HAL_UART_IRQHandler(&uart_dev_g[UART_1].uart_handler);
    _Uart_CMD_DRV(&uart_dev_g[UART_1]);
}

bool Get_Uart_LOG_D_Status(u8 index)
{
    return uart_dev_g[index].is_LOG_D_ON;
}

void Set_Uart_LOG_D_Status(u8 index, bool status)
{
    if (false == status) {
        memset(&(uart_dev_g[index].key_LOG_D[0]), 0, LOG_D_KEY_MAX);
    }
    uart_dev_g[index].is_LOG_D_ON = status;
}

bool Match_Uart_LOG_D_KEY(u8 index, const char *key)
{
    size_t len_key = strlen(key);
    size_t len_cache = strlen(&(uart_dev_g[index].key_LOG_D[0]));
    if (0 == ZSS_STRNCMP(key, (const char *)(&(uart_dev_g[index].key_LOG_D[0]))) && len_key == len_cache) {
        return true;
    } else {
        return false;
    }
}

void Set_Uart_LOG_D_KEY(u8 index, const char *key)
{
    memset(&(uart_dev_g[index].key_LOG_D[0]), 0, LOG_D_KEY_MAX);
    strlcpy(&(uart_dev_g[index].key_LOG_D[0]), key, LOG_D_KEY_MAX);
}

void Set_Uart_DBG_INT(u8 index, int val, u8 dbg_int_idx)
{
    uart_dev_g[index].dbg_int[dbg_int_idx] = val;
}

int Get_Uart_DBG_INT(u8 index, u8 dbg_int_idx)
{
    return uart_dev_g[index].dbg_int[dbg_int_idx];
}

void Clean_Uart_DBG_INT(u8 index)
{
    memset(&(uart_dev_g[index].dbg_int[0]), 0, DBG_INT_MAX * sizeof(int));
}

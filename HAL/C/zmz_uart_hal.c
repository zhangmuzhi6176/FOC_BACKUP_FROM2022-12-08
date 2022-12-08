#include "zmz_uart_fmwk.h"
#include "zmz_uart_hal.h"


void Uart_Cmd(u8 uart_index, const char *cmd)
{
    Uart_parse_one(uart_index, cmd);
}


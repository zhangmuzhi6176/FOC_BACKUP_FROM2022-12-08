#include "zmz_foc.h"
#include "zmz_uart_fmwk.h"
#include "zmz_uart_hal.h"
#include "string.h"
#include "stdlib.h"

void Uart_parse_one(u8 uart_index, const char *input)
{
    u8 len = strlen(input) + 1;
    char *whole = (char *)malloc(len);
    char *cmd = NULL;
    char *value_c = NULL;
    char *dbg_int_val = NULL;
    int value_i = 0;

    snprintf(whole, len, "%s", input);
    cmd = strtok_r(whole, ":", &value_c);
    value_i = atoi(value_c);
    ZSS_UNUSED(value_i);
    ZSS_UNUSED(dbg_int_val);

    if (!ZSS_STRNCMP("LOG_D", cmd)) {
        if (!ZSS_STRNCMP("OFF", value_c)) {
            Set_Uart_LOG_D_Status(uart_index, false);
        } else {
            Set_Uart_LOG_D_Status(uart_index, true);
            Set_Uart_LOG_D_KEY(uart_index, value_c);
        }
    } else if (!ZSS_STRNCMP("SVPWM", cmd)) {
        if (!ZSS_STRNCMP("SHUTDOWN", value_c)) {
            SVPWM_Disable(FOC_I);
        } else if (!ZSS_STRNCMP("TURNUP", value_c)) {
            SVPWM_Enable(FOC_I);
        }
    } else if (!ZSS_STRNCMP("FOC", cmd)) {
        if (!ZSS_STRNCMP("PRT_CUR", value_c)) {
            FOC_Print_Current(FOC_I);
        }
    } else if (!ZSS_STRNCMP("DBG_GET_INT", cmd)) {
        Clean_Uart_DBG_INT(uart_index);
        for (u8 i = 0; NULL != value_c; i++) {
            dbg_int_val = strtok_r(NULL, ":", &value_c);
            if (NULL == dbg_int_val) {
                break;
            }
            Set_Uart_DBG_INT(uart_index, atoi(dbg_int_val), i);
        }
    }else if (!ZSS_STRNCMP("cmd", cmd)) {
    } else {
        ZSS_UART_LOGI("INVALID CMD [%s]\r\n", input);
    }

    free(whole);
}

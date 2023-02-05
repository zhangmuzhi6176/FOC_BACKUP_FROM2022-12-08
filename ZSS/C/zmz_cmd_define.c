#include "zmz_cmd_define.h"
#include "zmz_system.h"

static u8 cmd_param_size[] = {
    [ZSS_CMD_FOC_CTRL] = sizeof(foc_ctrl_t),
};

u16 ZSS_Get_Cmd_Param_Size(ZSS_CMD_ID_e cmdid)
{
    return cmd_param_size[cmdid];
}

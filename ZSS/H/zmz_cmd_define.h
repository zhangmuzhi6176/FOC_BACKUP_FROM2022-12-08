#ifndef _ZMZ_CMD_DEFINE_H
#define _ZMZ_CMD_DEFINE_H

#include "zmz_system.h"
#pragma anon_unions

typedef enum ZSS_CMD_ID {
    ZSS_CMD_START = 0,
    ZSS_CMD_FOC_CTRL,                       /* @ foc_ctrl_t */

    ZSS_CMD_MAX,
} ZSS_CMD_ID_e;

/* --------------------------------------------------------------- CMD_PARAM_DEFINE --------------------------------------------------------------- */

typedef enum FOC_MODE_ID {
    FOC_KEEP_TORQ = 0,
    FOC_KEEP_SPEED,
    FOC_KEEP_POSITION,
    FOC_MODE_MAX,
} FOC_MODE_ID_e;

typedef struct foc_position {
    FOC_MODE_ID_e mode;
    union __attribute__ ((__packed__)) param {
        float torq;
        float speed;
        float mech_angle;
    };
} __attribute__ ((__packed__)) foc_ctrl_t;

u16 ZSS_Get_Cmd_Param_Size(ZSS_CMD_ID_e cmdid);

#endif

#ifndef _ZMZ_SYSTEM_H
#define _ZMZ_SYSTEM_H

typedef enum zss_return {
    SUCCESS_ZMZ = 0,
    COMMON_ERROR_ZMZ = -1001,
    TIME_OUT_ZMZ = -1002,
} zss_return_e;

typedef int s32;
typedef short s16;
typedef char s8;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef enum bool {
    false = 0,
    true = !false
} bool;

#define PI                                  3.1415926f
#define DEG_2_RAD(angle)                    ((angle) / 180.0 * PI)
#define RAD_2_DEG(rad)                      ((rad) * 180.0 / PI)

#define ZSS_OFFSET_OF(TYPE, MEMBER)         ((size_t) & ((TYPE *)0)->MEMBER)
#define ZSS_ARRAY_SIZE(arr)                 (sizeof(arr) / sizeof(arr[0]))
#define ZSS_INDEX_IN_ARRAY(arr, member)     (&(member) - &(arr[0]))

#define ZSS_STR(str)                        #str
#define ZSS_STRNCMP(str1, str2)             (strncmp(str1, str2, strlen(str1)) || !(strlen(str1) == strlen(str2)))

#define ZSS_UNUSED(x)                       ((x) = (x))

#define ZSS_ENABLE                          1
#define ZSS_DISABLE                         0

#endif

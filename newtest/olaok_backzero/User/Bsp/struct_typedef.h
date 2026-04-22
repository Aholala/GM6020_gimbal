#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/* 直接使用标准 stdint.h，避免与 ARM GCC 系统类型冲突 */
#include <stdint.h>

typedef unsigned char bool_t;
typedef float         fp32;
typedef double        fp64;

#endif
//
// Created by CYK on 2024/11/20.
//

#ifndef USER_LIB_H
#define USER_LIB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define NORMALIZATION_MAX 1

#define PI 3.14159265358979323846f
#define DEC_CON (float) PI/180

uint32_t Get_Time_us();
uint32_t Get_Time_ms();
float Get_Time_ms_us();
uint16_t unsigned_16(uint8_t *p);
float ABS(float target);

#ifdef __cplusplus
}
#endif


/**
 * @brief 知道一个结构体的子变量的地址，这个结构体的名称和这个子变量在结构体的名称，可以返回结构体变量的基地址
 * @param 结构体的子变量的地址
 * @param 结构体的名称
 * @param 子变量在结构体的名称
 */
#define Container_Of(ptr, type, member) ({ \
const typeof(((type *)0)->member) *__mptr = (ptr) ; \
(type *)((char *)__mptr - offsetof(type,member)) ;})

#define HIGH_BYTE(x) ((uint8_t)(((x) & 0xff00) >> 8))
#define LOW_BYTE(x) ((uint8_t)((x) & 0x00ff))
#define MERGE_BYTES(high, low) (((uint16_t) (high) << 8) | (uint16_t)(low))

#define VAL_LIMIT(val, min, max) \
do                         \
{                          \
if ((val) <= (min))    \
{                      \
(val) =(min);      \
}                      \
else if((val)>=(max))  \
{                      \
(val)=(max);       \
}                      \
\
} while(0)

#define ABS_LIMIT(val, limit) \
do                             \
{                              \
if ((val) <= -(limit))          \
{                            \
(val) = -(limit);             \
}                            \
else if ((val) >= (limit))     \
{                            \
(val) = (limit);             \
}                            \
} while (0)

void Remove_Subtle_Error(float *target,float error_min);


#endif //USER_LIB_H


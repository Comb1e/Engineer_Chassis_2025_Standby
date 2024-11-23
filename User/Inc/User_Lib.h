//
// Created by CYK on 2024/11/20.
//

#ifndef USER_LIB_H
#define USER_LIB_H

#include "stm32f4xx_hal.h"

#define NORMALIZATION_MAX 1

#define PI 3.14159265358979f
#define DEC_CON (float) PI/180

uint32_t Get_Time_us();
uint32_t Get_Time_ms();
float Get_Time_ms_us();
float ABS_Limit(float target,float val);
uint16_t unsigned_16(uint8_t *p);
float ABS(float target);

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


#endif //USER_LIB_H

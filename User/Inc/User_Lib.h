//
// Created by CYK on 2024/11/20.
//

#ifndef USER_LIB_H
#define USER_LIB_H

#include "stm32f4xx_hal.h"

#define NORMALIZATION_MAX 1

float ABS_Limit(float target,float val);

/**
 * @brief 知道一个结构体的子变量的地址，这个结构体的名称和这个子变量在结构体的名称，可以返回结构体变量的基地址
 * @param 结构体的子变量的地址
 * @param 结构体的名称
 * @param 子变量在结构体的名称
 */
#define container_of(ptr, type, member) ({ \
const typeof(((type *)0)->member) *__mptr = (ptr) ; \
(type *)((char *)__mptr - offsetof(type,member)) ;})


#endif //USER_LIB_H

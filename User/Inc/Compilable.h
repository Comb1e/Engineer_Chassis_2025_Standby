//
// Created by CYK on 2024/11/27.
//

#ifndef COMPILABLE_H
#define COMPILABLE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#pragma pack(1)
typedef struct
{
    uint32_t tx_id;
    uint8_t len;
    uint8_t *buf_data;
}can_tx_member_t;
#pragma pack()

void BSP_CAN_Init();

#ifdef __cplusplus
}
#endif

#endif //COMPILABLE_H

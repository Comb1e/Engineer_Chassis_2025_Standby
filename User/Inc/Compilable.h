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

typedef struct
{
    uint32_t tx_id;
    uint8_t *buf_data;
    uint32_t len;
}can_tx_member_t;

void BSP_CAN_Init();

//extern uint32_t debug;

#ifdef __cplusplus
}
#endif
#endif //COMPILABLE_H

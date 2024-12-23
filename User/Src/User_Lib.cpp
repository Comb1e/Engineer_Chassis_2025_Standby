//
// Created by CYK on 2024/11/20.
//

#include "User_Lib.h"
#include <string.h>

uint32_t Get_Time_us()
{
    return TIM7->CNT;
}

uint32_t Get_Time_ms()
{
    return HAL_GetTick();
}

float Get_Time_ms_us()
{
    return ((float)Get_Time_ms() + (float)Get_Time_us()/1000.0f);
}

uint16_t unsigned_16(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

float ABS(float target)
{
    if(target < 0)
    {
        return -target;
    }
    return target;
}

void Remove_Subtle_Error(float *target, float error_min)
{
    if(ABS(*target) <= error_min)
    {
        *target = 0.0f;
    }
}

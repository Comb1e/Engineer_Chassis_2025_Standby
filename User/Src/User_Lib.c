//
// Created by CYK on 2024/11/20.
//

#include "User_Lib.h"

float ABS_Limit(float target,float val)
{
    if(target > val)
    {
        return val;
    }
    if(target < -val)
    {
        return -val;
    }
    return target;
}

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
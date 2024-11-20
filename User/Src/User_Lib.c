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
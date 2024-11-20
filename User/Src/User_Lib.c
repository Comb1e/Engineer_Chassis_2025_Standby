//
// Created by CYK on 2024/11/20.
//

#include "User_Lib.h"

void ABS_Limit(float *target,float val)
{
    if(*target > val)
    {
        *target = val;
    }
    else if(*target < -val)
    {
        *target = -val;
    }
}
//
// Created by CYK on 2024/11/21.
//

#include "SPD_Plan.h"

#include <math.h>

#include "stdbool.h"
#include "User_Lib.h"

void Slope_Speed_Init(slope_speed_t *slope_speed, float out, float acc, float dec, float out_max, float target)
{
    slope_speed->acc = acc;
    slope_speed->dec = dec;
    slope_speed->target = target;
    slope_speed->out = out;
    slope_speed->out_max = out_max;
}

float Get_Slope_Speed(slope_speed_t *slope_speed)
{
    float target_speed = slope_speed->target;
    target_speed = ABS_Limit(target_speed, slope_speed->out_max);
    bool if_acc = true;
    int dir;
    if(slope_speed->out > 0)
    {
        dir = 1;
        if(target_speed < slope_speed->out)
        {
            if_acc = false;
        }
    }
    else
    {
        dir = -1;
        if(target_speed > slope_speed->out)
        {
            if_acc = false;
        }
    }

    float speed_err = fabsf(target_speed - slope_speed->out);
    float acc;
    if (if_acc)
    {
        acc = slope_speed->acc;
    }
    else
    {
        acc = -slope_speed->dec;
    }

    if (speed_err < fabsf(acc))
    {
        slope_speed->out = target_speed;
    }
    else
    {
        slope_speed->out = fabsf(slope_speed->out) + acc;        //不用计算出总步长，以增量式慢慢推进，走一步看一步
        slope_speed->out = slope_speed->out * (float) dir;
    }
    return slope_speed->out;
}

void Update_Slope_SPD(slope_speed_t *slope_speed, float acc, float dec, float out_max)
{
    slope_speed->acc = acc;
    slope_speed->dec = dec;
    slope_speed->out_max = out_max;
}

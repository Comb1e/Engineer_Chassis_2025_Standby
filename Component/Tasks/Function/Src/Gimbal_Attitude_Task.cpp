//
// Created by CYK on 2024/12/1.
//

#include "Gimbal_Attitude_Task.h"

#include "Drv_Gimbal.h"

#if GIMBAL_ATTITUDE

void Gimbal_Attitude_Task(void *argument)
{
    for(;;)
    {
        if(gimbal.Check_Enable())
        {
            if(gimbal.pitch_enable_flag)
            {
                gimbal.Update_Pitch_Control();
                debug++;
            }
            if(gimbal.yaw_enable_flag)
            {
                gimbal.Update_Yaw_Control();
            }
        }
        osDelay(1);
    }
}

#endif
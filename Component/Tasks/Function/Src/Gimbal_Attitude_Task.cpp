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
        if(small_gimbal.Check_Enable())
        {
            if(small_gimbal.pitch_enable_flag)
            {
                small_gimbal.Update_Pitch_Control();
            }
            if(small_gimbal.yaw_enable_flag)
            {
                small_gimbal.Update_Yaw_Control();
            }
        }
        osDelay(1);
    }
}

#endif
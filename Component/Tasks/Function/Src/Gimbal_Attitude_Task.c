//
// Created by CYK on 2024/11/25.
//

#include "Gimbal_Attitude_Task.h"
#include "cmsis_os2.h"
#include "Drv_Gimbal.h"
#include "Drv_RemoteCtrl.h"
#include "Global_CFG.h"

#if GIMBAL

void Gimbal_Attitude_Task(void *argument)
{
    for(;;)
    {
        if (rc.ctrl_protection.connect_flag)
        {
            Gimbal_Update_Yaw_Control(&gimbal);
        }
        osDelay(1);
    }
}

#endif
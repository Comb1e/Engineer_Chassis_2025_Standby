//
// Created by CYK on 2024/12/8.
//

#include "AutoCtrl_Task.h"
#include "Drv_Robot.h"

bool reset_flag = false;
bool auto_exchange_flag = false;
void AutoCtrl_Task(void *argument)
{
    osDelay(2000);
    g_robot.PTR_Init(&g_chassis,&g_gimbal,&usb,&g_arm,&g_info,&g_absorb);
    for(;;)
    {
        g_robot.Update_Chassis_Speed_Limit();
        g_robot.Check_KB_Event();
        g_robot.Check_Rot();
        g_robot.Check_Death();
        g_robot.Check_Error();
        g_robot.Update_Visual_Exchange();

        if(reset_flag)
        {
            kb.Set_Gimbal_Reset();
            reset_flag = false;
        }

        if(auto_exchange_flag)
        {
            auto_exchange_flag = false;
            g_robot.CreatTask_Auto_Exchange();
        }

        osDelay(2);
    }
}
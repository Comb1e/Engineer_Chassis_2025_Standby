//
// Created by CYK on 2024/12/8.
//

#include "AutoCtrl_Task.h"
#include "Drv_Robot.h"

bool reset_flag = false;
bool sucker_dir_move_flag = false;
bool auto_exchange_flag = false;
bool arm_move_flag = false;
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

        if(sucker_dir_move_flag)
        {
            g_robot.arm->Sucker_Dir_Move(200.0f,0.2f);
            sucker_dir_move_flag = false;
        }

        if(arm_move_flag)
        {
            g_robot.arm->Add_Point_Target_Pos_Vel(X,g_robot.usb->ore_to_target_pose.x,0.5f);
            g_robot.arm->Add_Point_Target_Pos_Vel(Y,g_robot.usb->ore_to_target_pose.y,0.5f);
            //g_robot.arm->Add_Point_Target_Pos_Vel(Z,g_robot.usb->ore_to_target_pose.z,0.5f);

            arm_move_flag = false;
        }

        osDelay(2);
    }
}
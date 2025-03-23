//
// Created by CYK on 2024/12/8.
//

#include "AutoCtrl_Task.h"
#include "Drv_Robot.h"

bool reset_flag = false;
bool auto_exchange_flag = false;
bool auto_small_island_flag = false;
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
            //g_robot.Gimbal_Reset();
            g_robot.Arm_Homing();
            reset_flag = false;
        }

        if(auto_exchange_flag)
        {
            auto_exchange_flag = false;
            g_robot.control_mode = VISUAL_CONTROL;
            g_robot.Exchange_Before_Getting_In();
            g_robot.Exchange_Before_Getting_In_Adjust();

            g_robot.Exchange_Getting_In();
            g_robot.absorb->Set_Sucker_Close(ARM_SUCKER);
            osDelay(1000);
            g_robot.Exchange_Back();
            g_robot.gimbal->Set_Homing();
            g_robot.usb->target_rx_flag = true;;
            g_robot.control_mode = RC_KB_CONTROL;
        }

        if(auto_small_island_flag)
        {
            auto_small_island_flag = false;
            g_robot.control_mode = AUTO_CONTROL;
            g_robot.Pre_For_Auto_SmallIsland_Or_GroundMine();
            g_robot.SmallIsland_Or_GroundMine_Pre();
            g_robot.SmallIsland_Or_GroundMine_1();
            g_robot.SmallIsland_Or_GroundMine_Touching();
            g_robot.SmallIsland_Lay();

            g_robot.absorb->Get_Ore_State()->Set_Ore_Source(SMALL_ISLAND);

            g_robot.SmallIsland_Or_GroundMine_Pre_Back();
            g_robot.Arm_Homing();

            g_robot.chassis->need_flag = true;
            g_robot.Set_Auto_Situation(Auto_None);
            g_robot.control_mode = RC_KB_CONTROL;
        }

        osDelay(2);
    }
}
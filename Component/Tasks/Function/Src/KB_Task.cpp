//
// Created by CYK on 2024/11/29.
//

#include "KB_Task.h"
#include "Drv_Keyboard.h"
#include "cmsis_os2.h"
#include "Drv_Robot.h"

void KB_State_Task(void *argument)
{
    osDelay(2000);
    for(;;)
    {
        if(g_robot.enable_flag && g_robot.Check_Control_Mode_RC_KB_CONTROL())
        {
            kb.Check_Mouse_State();
            kb.Check_RC_State();
            kb.Check_KB_State();
        }
        else
        {
#if VISUAL_CONTROL_TEST
            if(RC_Check_Wheel_State(RC_WHEEL_UP))
            {
                g_absorb.Set_Sucker_Open(ARM_SUCKER);
            }
            else if(RC_Check_Wheel_State(RC_WHEEL_DOWN))
            {
                g_absorb.Set_Sucker_Close(ARM_SUCKER);
            }
#endif
        }
        osDelay(KB_CONTROL_CYCLE);
    }
}

void KB_Event_Task(void *argument)
{
    for(;;)
    {
        if(g_robot.enable_flag)
        {
            kb.Check_Mouse_Event();
            kb.Check_KB_FG_Event();
            if(g_robot.Check_Control_Mode_RC_KB_CONTROL())
            {
                kb.Check_RC_Event();
                kb.Check_KB_Event();
            }
        }
        osDelay(1);
    }
}
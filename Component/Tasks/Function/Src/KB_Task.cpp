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
        if(robot.enable_flag)
        {
            kb.Check_Mouse_State();
            kb.Check_RC_State();
            kb.Check_KB_State();
        }
        else
        {
            continue;
        }
        osDelay(KB_CONTROL_CYCLE);
    }
}

void KB_Event_Task(void *argument)
{
    for(;;)
    {
        if(robot.enable_flag)
        {
            kb.Check_Mouse_Event();
            kb.Check_RC_Event();
            kb.Check_KB_Event();
        }
        else
        {
            continue;
        }
        osDelay(1);
    }
}
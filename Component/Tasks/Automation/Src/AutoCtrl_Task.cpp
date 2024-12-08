//
// Created by CYK on 2024/12/8.
//

#include "AutoCtrl_Task.h"
#include "Drv_Robot.h"

void AutoCtrl_Task(void *argument)
{
    for(;;)
    {
        robot.Update_Chassis_Speed_Limit();
        robot.Check_KB_Event();
        osDelay(2);
    }
}
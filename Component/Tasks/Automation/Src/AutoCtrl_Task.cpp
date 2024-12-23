//
// Created by CYK on 2024/12/8.
//

#include "AutoCtrl_Task.h"
#include "Drv_Robot.h"

void AutoCtrl_Task(void *argument)
{
    osDelay(2000);
    robot.PTR_Init(&chassis,&gimbal,&usb,&arm,&info,&absorb);
    for(;;)
    {
        robot.Update_Chassis_Speed_Limit();
        robot.Check_KB_Event();
        robot.Check_Rot();
        robot.Check_Death();
        robot.Check_Error();
        robot.Update_Visual_Exchange();
        osDelay(2);
    }
}
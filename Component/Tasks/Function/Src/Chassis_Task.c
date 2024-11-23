//
// Created by CYK on 2024/11/20.
//

#include "Chassis_Task.h"
#include "cmsis_os2.h"
#include "Drv_Chassis.h"
#include "main.h"

void Chassis_Task(void *argument)
{
    Chassis_Task_Init();
    for(;;)
    {
        //i++;
        osDelay(1);
    }
}

void Chassis_Task_Init()
{
    Chassis_Init();
}
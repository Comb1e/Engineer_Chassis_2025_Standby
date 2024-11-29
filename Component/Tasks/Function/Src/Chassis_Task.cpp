//
// Created by CYK on 2024/11/27.
//

#include "Chassis_Task.h"
#include "Global_CFG.h"
#include "cmsis_os2.h"
#include "RTOS.h"
#include "Drv_Chassis.h"

#if CHASSIS

void Chassis_Task(void *argument)
{
    chassis.Init();
    while(!chassis.Check_Init_Completely())
    {
        osDelay(1);
    }
    for(;;)
    {
        chassis.Update_Enable_Flag();
        chassis.Update_Ready();
        if(chassis.Check_Ready_Flag() && chassis.Check_Enable_Flag())
        {
            if(chassis.Check_Can_Use())
            {

            }
            else
            {
                continue;
            }
        }
        else
        {
            chassis.Set_Free();
        }
        osDelay(1);
    }
}

#endif
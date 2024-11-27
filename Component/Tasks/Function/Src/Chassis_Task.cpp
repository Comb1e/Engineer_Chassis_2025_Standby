//
// Created by CYK on 2024/11/27.
//

#include "Chassis_Task.h"
#include "Global_CFG.h"
#include "cmsis_os2.h"
#include "RTOS.h"

#if CHASSIS

void Chassis_Task(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}

#endif
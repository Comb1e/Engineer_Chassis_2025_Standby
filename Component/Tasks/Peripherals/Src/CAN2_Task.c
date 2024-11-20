//
// Created by CYK on 2024/11/20.
//

#include "CAN2_Task.h"
#include "BSP_Can.h"
#include "cmsis_os2.h"

void CAN2_Task(void *argument)
{
    CAN2_Init();
    for(;;)
    {
        osDelay(1);
    }
}
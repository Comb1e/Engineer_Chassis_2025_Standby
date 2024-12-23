//
// Created by CYK on 2024/12/2.
//

#include "Check_Communicate_Task.h"

#include "Drv_Absorb.h"
#include "Global_CFG.h"

void Check_Communicate_Task(void *argument)
{
    osDelay(2000);

    for(;;)
    {
        osDelay(6);
        absorb.Check_Pump_MCU_For_Loss();
    }
}
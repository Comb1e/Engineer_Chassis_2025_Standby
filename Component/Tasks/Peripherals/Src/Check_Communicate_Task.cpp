//
// Created by CYK on 2024/12/2.
//

#include "Check_Communicate_Task.h"
#include "Drv_Absorb.h"
#include "Drv_Communication.h"

void Check_Communicate_Task(void *argument)
{
    osDelay(2000);
    for(;;)
    {
        gc_communication.Check_Connect();
        absorb.Check_Pump_MCU_For_Loss();
        osDelay(6);
    }
}
//
// Created by CYK on 2024/12/2.
//

#include "Check_Communicate_Task.h"

#include "Drv_Absorb.h"
#include "Global_CFG.h"
#include "Drv_Info.h"
#include "Drv_Arm.h"

void Check_Communicate_Task(void *argument)
{
    osDelay(2000);
    osSemaphoreAcquire(ArmResetInitBinarySemHandle, 10);
    osSemaphoreAcquire(ArmUpdateBinarySemHandle, 10);
    for(;;)
    {
        osDelay(6);
        g_absorb.Check_Pump_MCU_For_Loss();
        g_info.Check_Lost();
        g_arm.Check_Lost();
    }
}
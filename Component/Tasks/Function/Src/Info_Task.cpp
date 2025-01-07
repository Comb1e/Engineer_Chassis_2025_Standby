//
// Created by CYK on 2024/12/2.
//

#include "Info_Task.h"
#include "Drv_Info.h"
#include "cmsis_os2.h"
#include "Global_CFG.h"

#if INFO

void Info_Task(void *argument)
{
    g_info.Init(INFO_CAN,INFO_RX_CAN2_STDID,INFO_TX_CAN2_STDID,ArmResetInitBinarySemHandle);
    osSemaphoreAcquire(ArmResetInitBinarySemHandle, 0);
    g_info.Update_Data();
    g_info.CAN_Send_MSG();
    for(;;)
    {
        g_info.Update_Enable();
        if(g_info.Check_Enable())
        {
            g_info.Update_Data();
            g_info.CAN_Send_MSG();
        }
        osDelay(2);
    }
}

#endif
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
    info.Init(INFO_CAN,INFO_RX_CAN2_STDID,INFO_TX_CAN2_STDID,ArmResetInitBinarySemHandle);
    osSemaphoreAcquire(ArmResetInitBinarySemHandle, 0);
    info.Update_Data();
    info.CAN_Send_MSG();
    for(;;)
    {
        info.Update_Enable();
        if(info.Check_Enable() && info.Check_Connect_Flag())
        {
            info.Update_Data();
            info.CAN_Send_MSG();
        }
        osDelay(2);
    }
}

#endif
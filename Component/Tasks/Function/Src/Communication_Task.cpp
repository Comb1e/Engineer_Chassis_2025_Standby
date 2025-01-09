//
// Created by CYK on 2024/12/23.
//

#include "Communication_Task.h"
#include "Drv_Communication.h"

void Communication_Task(void *argument)
{
    gc_communication.Init(CHASSIS_TO_GIMBAL_TX_CAN2_STDID,CHASSIS_TO_GIMBAL_RX_CAN2_STDID,GimbalToChassisUpdateBinarySemHandle,GIMBAL_TO_CHASSIS_HCAN);
    osSemaphoreAcquire(GimbalToChassisUpdateBinarySemHandle,osWaitForever);
    for(;;)
    {
        gc_communication.Check_Connect();
        if(gc_communication.connect_flag)
        {
            gc_communication.All_Set_Enable();
            gc_communication.Update_Data();
        }
        else
        {
            gc_communication.All_Set_Free();
        }
        osDelay(3);
    }
}
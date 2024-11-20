//
// Created by CYK on 2024/11/20.
//

#include "CAN1_Task.h"
#include "BSP_Can.h"
#include "cmsis_os2.h"
#include "RTOS.h"

void CAN1_Task(void *argument)
{
    CAN1_Task_Init();
    for(;;)
    {
        osSemaphoreAcquire(CAN1CountingSemHandle, osWaitForever);

        osDelay(1);
    }
}

void CAN1_Task_Init()
{
    CAN1_Init();
    CAN1_Tx_Init();
}
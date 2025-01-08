//
// Created by CYK on 2024/11/22.
//
//名字应该是hi229
#include "HI299UM_Task.h"
#include "cmsis_os2.h"
#include "Drv_HI229UM.h"
#include "RTOS.h"
#include "usart.h"
#include "Global_CFG.h"

void HI229UM_Task(void *argument)
{
    static osStatus_t s_stat;

    HI229UM_Task_Init();
    for (;;)
    {
        HI229UM_Receive_DMA();
        s_stat = osSemaphoreAcquire(HI229UMRxBinarySemHandle, 20);
        if (s_stat == osOK)
        {
            HI229UM_Set_Connect();
            if (HI229UM_Check_Legal())
            {
                HI229UM_Update_Data();
            }
        }
        else
        {
            HI229UM_Set_Lost();
        }
        HI229UM_Update_Ready();
        osDelay(1);
    }
}

void HI229UM_Task_Init()
{
    HI229UM_Init(&HI229UM_UART);
    osSemaphoreAcquire(HI229UMRxBinarySemHandle, 0);
    HAL_UART_RegisterRxEventCallback(&HI229UM_UART, HI229UM_RxCallBack);
    HI229UM_Set_Nine_Axis_Mode();
}

void HI229UM_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size)
{
    osSemaphoreRelease(HI229UMRxBinarySemHandle);
}
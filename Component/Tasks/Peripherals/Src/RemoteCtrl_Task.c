//
// Created by CYK on 2024/11/20.
//

#include "RemoteCtrl_Task.h"
#include "Drv_RemoteCtrl.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "RTOS.h"

void RemoteCtrl_Task(void *argument)
{
    static osStatus_t status;
    RemoteCtrl_Task_Init();
    for(;;)
    {
        status = osSemaphoreAcquire(RCUpdateBinarySemHandle, 20);
        if(status == osOK)
        {
            RC_Set_Connect();
            RC_UpdateData();
            RC_UpdateEvent();
        }
        else
        {
            RC_Lost_CntDown();
        }
        RC_CheckConnection();
        osDelay(1);
    }
}

void RemoteCtrl_Task_Init()
{
    rc.huart = &huart1;
    HAL_UART_RegisterRxEventCallback(rc.huart, RC_RxCallBack);

    rc.ctrl_protection.connect_flag = false;

    osSemaphoreAcquire(RCUpdateBinarySemHandle, 0);
}

void RC_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size)
{
    osSemaphoreRelease(RCUpdateBinarySemHandle);
}

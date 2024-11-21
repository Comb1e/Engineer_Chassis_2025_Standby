//
// Created by CYK on 2024/11/20.
//

#include "CAN1_Task.h"
#include "BSP_Can.h"
#include "cmsis_os2.h"
#include "RTOS.h"
#include "can.h"

void CAN1_Task(void *argument)
{
    CAN1_Task_Init();
    can_tx_member_t can_tx_member;
    uint32_t tx_mailbox;
    for(;;)
    {
        osSemaphoreAcquire(CAN1CountingSemHandle, osWaitForever);
        osMessageQueueGet(CAN1SendQueueHandle, &can_tx_member, 0, osWaitForever);
        can1_tx_header.StdId = can_tx_member.can_id;
        HAL_CAN_AddTxMessage(&hcan1,&can1_tx_header,can_tx_member.can_tx_buff,&tx_mailbox);
        osDelay(1);
    }
}

void CAN1_Task_Init()
{
    CAN1_Init();
    CAN1_Tx_Init();
}
//
// Created by CYK on 2024/11/20.
//

#include "CAN2_Task.h"
#include "BSP_Can.h"
#include "cmsis_os2.h"
#include "can.h"
#include "RTOS.h"

void CAN2_Task(void *argument)
{
    CAN2_Task_Init();
    can_tx_member_t can_tx_member;
    uint32_t tx_mailbox;
    for(;;)
    {
        osSemaphoreAcquire(CAN2CountingSemHandle, osWaitForever);
        osMessageQueueGet(CAN2SendQueueHandle, &can_tx_member, 0, osWaitForever);
        can2_tx_header.StdId = can_tx_member.can_id;
        HAL_CAN_AddTxMessage(&hcan2,&can2_tx_header,can_tx_member.can_tx_buff,&tx_mailbox);
        osDelay(1);
    }
}

void CAN2_Task_Init()
{
    CAN2_Init();
    CAN2_Tx_Init();
    CAN2_Filter_Init();
}
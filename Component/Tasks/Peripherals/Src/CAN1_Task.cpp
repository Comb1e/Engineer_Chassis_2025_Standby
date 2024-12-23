//
// Created by CYK on 2024/11/28.
//

#include "CAN1_Task.h"
#include "can.h"
#include "compilable.h"
#include "RTOS.h"

void CAN1_Task(void *argument)
{
    uint32_t tx_mailbox;
    can_tx_member_t tx_member;
    CAN_TxHeaderTypeDef can1_tx_header;
    can1_tx_header.RTR = CAN_RTR_DATA;
    can1_tx_header.IDE = CAN_ID_STD;
    for(;;)
    {
        osSemaphoreAcquire(CAN1CountingSemHandle,osWaitForever);
        osMessageQueueGet(CAN1SendQueueHandle,&tx_member,0,osWaitForever);
        can1_tx_header.StdId = tx_member.tx_id;
        can1_tx_header.DLC = tx_member.len;
        HAL_CAN_AddTxMessage(&hcan1,&can1_tx_header,tx_member.buf_data,&tx_mailbox);
    }
}

//
// Created by CYK on 2024/11/28.
//

#include "CAN2_Task.h"
#include "can.h"
#include "cmsis_os2.h"
#include "compilable.h"
#include "RTOS.h"

void CAN2_Task(void *argument)
{
    can_tx_member_t tx_member;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for(;;)
    {
        osSemaphoreAcquire(CAN2CountingSemHandle,osWaitForever);
        osMessageQueueGet(CAN2SendQueueHandle,&tx_member,NULL,osWaitForever);
        tx_header.StdId = tx_member.tx_id;
        tx_header.DLC = tx_member.len;
        HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_member.buf_data,&tx_mailbox);
        osDelay(1);
    }
}
